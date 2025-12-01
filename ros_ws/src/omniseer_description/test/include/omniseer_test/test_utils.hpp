// Test utility functions
#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <limits>
#include <vector>

#include "omniseer/frontier.hpp"
#include "omniseer/grid_io.hpp"
#include "omniseer/grid_utils.hpp"

namespace omniseer_test
{
  namespace fs = std::filesystem;

  // Generates artifact directory for current test
  inline fs::path artifact_dir()
  {
    fs::path        root = fs::path(OMNISEER_ARTIFACTS_DIR); // CMake defined
    const auto*     ti   = ::testing::UnitTest::GetInstance()->current_test_info();
    fs::path        dir  = root / ti->test_suite_name() / ti->name();
    std::error_code ec;
    fs::create_directories(dir, ec);
    return dir;
  }

  // Simple color image writer from RGB buffer (row-major, 3 bytes per pixel)
  inline void write_ppm_rgb(const fs::path& p, int W, int H, const std::vector<uint8_t>& rgb)
  {
    std::ofstream f(p, std::ios::binary);
    ASSERT_TRUE(f.good()) << "open " << p.string();
    f << "P6\n" << W << " " << H << "\n255\n";
    f.write(reinterpret_cast<const char*>(rgb.data()), static_cast<std::streamsize>(rgb.size()));
    ASSERT_TRUE(f.good()) << "short write " << p.string();
  }

  inline void rgb_from_gray_bg(const omniseer::GridU8& g, std::vector<uint8_t>& rgb)
  {
    rgb.resize(static_cast<size_t>(g.width) * static_cast<size_t>(g.height) * 3u);
    for (size_t i = 0, j = 0; i < g.data.size(); ++i, j += 3)
    {
      const uint8_t gray = g.data[i];
      rgb[j + 0]         = gray;
      rgb[j + 1]         = gray;
      rgb[j + 2]         = gray;
    }
  }

  inline void set_px(std::vector<uint8_t>& rgb, int W, int H, int x, int y, uint8_t r, uint8_t g,
                     uint8_t b)
  {
    if (x < 0 || y < 0 || x >= W || y >= H)
      return;
    const size_t j =
        static_cast<size_t>(y) * static_cast<size_t>(W) * 3u + static_cast<size_t>(x) * 3u;
    rgb[j + 0] = r;
    rgb[j + 1] = g;
    rgb[j + 2] = b;
  }

  inline bool is_grayscale_at(const std::vector<uint8_t>& rgb, int W, int H, int x, int y)
  {
    if (x < 0 || y < 0 || x >= W || y >= H)
      return false;
    const size_t j =
        static_cast<size_t>(y) * static_cast<size_t>(W) * 3u + static_cast<size_t>(x) * 3u;
    return (rgb[j + 0] == rgb[j + 1]) && (rgb[j + 1] == rgb[j + 2]);
  }

  inline void stamp_disk(std::vector<uint8_t>& rgb, int W, int H, int cx, int cy, int rad,
                         uint8_t r = 255, uint8_t g = 255, uint8_t b = 0)
  {
    const int r2 = rad * rad;
    for (int dy = -rad; dy <= rad; ++dy)
      for (int dx = -rad; dx <= rad; ++dx)
      {
        if (dx * dx + dy * dy > r2)
          continue;
        set_px(rgb, W, H, cx + dx, cy + dy, r, g, b);
      }
  }

  struct RayStats
  {
    int ig_unique{0};
    int rays_contrib{0};
    int rays_no_contrib{0};
  };

  // DDA visitor matching the implementation in src/ray_cast.cpp
  template <typename Visitor>
  inline void visit_segment_cells_dda(const omniseer::GridU8& g, double ax, double ay, double bx,
                                      double by, Visitor&& visit)
  {
    using namespace omniseer::grid;

    int x = 0, y = 0;
    if (!world_to_cell(g, ax, ay, x, y))
      return;

    const double dx    = bx - ax;
    const double dy    = by - ay;
    const int    stepx = (dx > 0) - (dx < 0);
    const int    stepy = (dy > 0) - (dy < 0);

    const double INF     = std::numeric_limits<double>::infinity();
    const double tDeltaX = (dx != 0.0) ? (g.resolution / std::abs(dx)) : INF;
    const double tDeltaY = (dy != 0.0) ? (g.resolution / std::abs(dy)) : INF;

    auto next_x_boundary = [&]()
    {
      const int ix_next = x + (stepx > 0);
      return g.origin_x + static_cast<double>(ix_next) * g.resolution;
    };
    auto next_y_boundary = [&]()
    {
      const int iy_next = y + (stepy > 0);
      return g.origin_y + static_cast<double>(iy_next) * g.resolution;
    };

    double tMaxX = (dx != 0.0) ? ((next_x_boundary() - ax) / dx) : INF;
    double tMaxY = (dy != 0.0) ? ((next_y_boundary() - ay) / dy) : INF;

    double t = 0.0;
    while (in_bounds(x, y, g.width, g.height) && t <= 1.0)
    {
      if (tMaxX < tMaxY)
      {
        t = tMaxX;
        tMaxX += tDeltaX;
        x += stepx;
      }
      else if (tMaxY < tMaxX)
      {
        t = tMaxY;
        tMaxY += tDeltaY;
        y += stepy;
      }
      else
      {
        t = tMaxX;
        tMaxX += tDeltaX;
        tMaxY += tDeltaY;
        x += stepx;
        y += stepy;
      }

      if (!in_bounds(x, y, g.width, g.height) || t > 1.0)
        break;
      if (!visit(x, y))
        break;
    }
  }

  // Draw a fan of rays from origin; color per-cell along each ray:
  //  - Green when the cell is UNKNOWN and contributes (first time seen globally)
  //  - Magenta on segments that do not contribute (free or UNKNOWN already seen)
  //  - Red on the first occluder cell (and stop)
  // Also return IG and per-ray contribution stats.
  inline RayStats draw_rays_overlay_for_goal(const omniseer::GridU8& g, const omniseer::Params& p,
                                             const omniseer::Pose2D& origin,
                                             std::vector<uint8_t>&   rgb)
  {
    using namespace omniseer::grid;

    RayStats stats{};
    if (g.width <= 0 || g.height <= 0)
      return stats;

    // Prepare seen bitmap
    const size_t         n = static_cast<size_t>(g.width) * static_cast<size_t>(g.height);
    std::vector<uint8_t> seen(n, 0);

    const int    num_rays    = std::max(1, p.ray_cast_params.num_rays);
    const double max_ray_len = std::max(0.0, p.ray_cast_params.max_ray_len);
    if (max_ray_len <= 0.0)
      return stats;

    const double step_ang = (2.0 * M_PI) / static_cast<double>(num_rays);

    for (int k = 0; k < num_rays; ++k)
    {
      const double th = step_ang * static_cast<double>(k);
      const double ex = origin.x + max_ray_len * std::cos(th);
      const double ey = origin.y + max_ray_len * std::sin(th);

      int contributed_this_ray = 0;

      visit_segment_cells_dda(g, origin.x, origin.y, ex, ey,
                              [&](int x, int y) -> bool
                              {
                                const int     lin = idx(x, y, g.width);
                                const uint8_t v   = g.data[lin];

                                if (is_unknown(v, p))
                                {
                                  if (!seen[lin])
                                  {
                                    // First time this unknown is seen: contributes
                                    seen[lin] = 1u;
                                    ++stats.ig_unique;
                                    ++contributed_this_ray;
                                    set_px(rgb, g.width, g.height, x, y, 32, 220, 32); // green
                                  }
                                  else
                                  {
                                    // Already counted elsewhere: non-contributing
                                    if (is_grayscale_at(rgb, g.width, g.height, x, y))
                                      set_px(rgb, g.width, g.height, x, y, 200, 0, 200); // magenta
                                  }
                                  return true; // continue through unknown
                                }
                                if (is_occluder(v, p))
                                {
                                  set_px(rgb, g.width, g.height, x, y, 255, 32, 32); // red stop
                                  return false;                                      // stop
                                }
                                // Free/traversable: non-contributing segment
                                if (is_grayscale_at(rgb, g.width, g.height, x, y))
                                  set_px(rgb, g.width, g.height, x, y, 200, 0, 200); // magenta
                                return true;
                              });

      if (contributed_this_ray > 0)
        ++stats.rays_contrib;
      else
        ++stats.rays_no_contrib;
    }

    return stats;
  }

  // Writes the given bytes into a standard P5 PGM grayscale image file
  inline void write_pgm(const fs::path& p, uint32_t w, uint32_t h,
                        const std::vector<uint8_t>& bytes)
  {
    std::ofstream f(p, std::ios::binary);
    ASSERT_TRUE(f.good()) << "open " << p.string();
    f << "P5\n" << w << " " << h << "\n255\n";
    f.write(reinterpret_cast<const char*>(bytes.data()), bytes.size());
    ASSERT_TRUE(f.good()) << "short write " << p.string();
  }

  // Writes a PPM color image overlaying a red mask on a grayscale grid
  inline void write_ppm_overlay(const fs::path& p, const omniseer::GridU8& g,
                                const std::vector<uint8_t>& mask01)
  {
    std::ofstream f(p, std::ios::binary);
    ASSERT_TRUE(f.good()) << "open " << p.string();
    f << "P6\n" << g.width << " " << g.height << "\n255\n";
    for (size_t i = 0; i < mask01.size(); ++i)
    {
      uint8_t gray = g.data[i];
      uint8_t r = gray, gg = gray, b = gray;
      if (mask01[i])
      {
        r  = 255;
        gg = 16;
        b  = 16;
      }
      f.put(char(r));
      f.put(char(gg));
      f.put(char(b));
    }
    ASSERT_TRUE(f.good());
  }

  inline std::array<uint8_t, 3> hsv2rgb(double h, double s, double v)
  {
    // h in [0,1), s,v in [0,1]
    double c  = v * s;
    double hp = h * 6.0;
    double x  = c * (1.0 - std::fabs(std::fmod(hp, 2.0) - 1.0));
    double r = 0, g = 0, b = 0;
    if (0.0 <= hp && hp < 1.0)
    {
      r = c;
      g = x;
      b = 0;
    }
    else if (1.0 <= hp && hp < 2.0)
    {
      r = x;
      g = c;
      b = 0;
    }
    else if (2.0 <= hp && hp < 3.0)
    {
      r = 0;
      g = c;
      b = x;
    }
    else if (3.0 <= hp && hp < 4.0)
    {
      r = 0;
      g = x;
      b = c;
    }
    else if (4.0 <= hp && hp < 5.0)
    {
      r = x;
      g = 0;
      b = c;
    }
    else
    {
      r = c;
      g = 0;
      b = x;
    }
    double m   = v - c;
    auto   to8 = [&](double u)
    {
      u = (u + m);
      u = std::max(0.0, std::min(1.0, u));
      return (uint8_t) std::lround(u * 255.0);
    };
    return {to8(r), to8(g), to8(b)};
  }

  // Distinct deterministic color per component using the golden ratio.
  inline std::array<uint8_t, 3> color_for(int id)
  {
    // 0.618... evenly fills the circle; add an offset for nicer starting hue
    constexpr double PHI = 0.6180339887498949;
    double           h   = std::fmod(0.15 + id * PHI, 1.0); // hue ∈ [0,1)
    double           s   = 0.80;                            // saturation
    double           v   = 0.95;                            // value/brightness
    return hsv2rgb(h, s, v);
  }

  // Row-major linear index helper: y * W + x
  inline int I(int x, int y, int W)
  {
    return y * W + x;
  }

  // Writes a PPM color image where each pixel is colored based on its label ID
  inline void write_labels_ppm(const fs::path& p, const omniseer::GridU8& g,
                               const std::vector<int32_t>& labels)
  {
    std::ofstream f(p, std::ios::binary);
    ASSERT_TRUE(f.good()) << "open " << p.string();
    f << "P6\n" << g.width << " " << g.height << "\n255\n";
    for (size_t i = 0; i < labels.size(); ++i)
    {
      int32_t L = labels[i];
      uint8_t r = 0, gg = 0, b = 0;
      if (L >= 0)
      {
        auto c = color_for(L);
        r      = c[0];
        gg     = c[1];
        b      = c[2];
      }
      f.put(char(r));
      f.put(char(gg));
      f.put(char(b));
    }
    ASSERT_TRUE(f.good());
  }

  // Writes ranked goal pixels using a yellow brightness gradient (bright = high rank)
  inline void write_rank_gradient_ppm(const fs::path& p, const omniseer::GridU8& g,
                                      const std::vector<int32_t>& ranks)
  {
    std::ofstream f(p, std::ios::binary);
    ASSERT_TRUE(f.good()) << "open " << p.string();
    f << "P6\n" << g.width << " " << g.height << "\n255\n";

    int32_t max_rank = -1;
    for (int32_t r : ranks)
      if (r >= 0)
        max_rank = std::max(max_rank, r);
    const double denom = max_rank > 0 ? static_cast<double>(max_rank) : 1.0;

    for (size_t i = 0; i < ranks.size(); ++i)
    {
      int32_t rank = ranks[i];
      uint8_t r = 0, gg = 0, b = 0;
      if (rank >= 0 && max_rank >= 0)
      {
        double norm = 1.0 - static_cast<double>(rank) / denom; // 1.0 for best, 0.0 for worst
        norm        = std::clamp(norm, 0.0, 1.0);
        const double intensity = 0.25 + 0.75 * norm; // keep darkest dots visible
        r                      = static_cast<uint8_t>(std::lround(255.0 * intensity));
        gg                     = static_cast<uint8_t>(std::lround(255.0 * intensity));
        b                      = 0;
      }
      f.put(char(r));
      f.put(char(gg));
      f.put(char(b));
    }
    ASSERT_TRUE(f.good());
  }

  // Creates a GridU8 object with specified width, height, and optional cell data
  inline omniseer::GridU8 mk_grid(int w, int h, const std::vector<uint8_t>& cells = {})
  {
    omniseer::GridU8 g;
    g.width      = w;
    g.height     = h;
    g.resolution = 1.0f;
    g.data       = cells.empty() ? std::vector<uint8_t>(size_t(w * h), 0) : cells;
    return g;
  }

} // namespace omniseer_test
