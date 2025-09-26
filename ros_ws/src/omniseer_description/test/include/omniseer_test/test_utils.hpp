// Test utility functions, mostly for generating artifacts
#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <vector>

#include "omniseer/grid_io.hpp"

namespace omniseer_test
{
  namespace fs = std::filesystem;

  /*
  Generates artifact directory for current test
  */
  inline fs::path artifact_dir()
  {
    fs::path        root = fs::path(OMNISEER_ARTIFACTS_DIR); // CMake defined
    const auto*     ti   = ::testing::UnitTest::GetInstance()->current_test_info();
    fs::path        dir  = root / ti->test_suite_name() / ti->name();
    std::error_code ec;
    fs::create_directories(dir, ec);
    return dir;
  }

  /*
  Writes the given bytes into a standard P5 PGM grayscale image file
  */
  inline void write_pgm(const fs::path& p, uint32_t w, uint32_t h,
                        const std::vector<uint8_t>& bytes)
  {
    std::ofstream f(p, std::ios::binary);
    ASSERT_TRUE(f.good()) << "open " << p.string();
    f << "P5\n" << w << " " << h << "\n255\n";
    f.write(reinterpret_cast<const char*>(bytes.data()), bytes.size());
    ASSERT_TRUE(f.good()) << "short write " << p.string();
  }

  /*
  Writes a PPM color image overlaying a red mask on a grayscale grid
  */
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
      } // punchy red
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

  /*
  Writes a PPM color image where each pixel is colored based on its label ID
  */
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

  /*
  Creates a GridU8 object with specified width, height, and optional cell data (defaults to zeros)
  */
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
