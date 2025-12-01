#include "omniseer/ray_cast.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "omniseer/frontier.hpp"
#include "omniseer/grid_utils.hpp"

namespace omniseer
{

  using namespace omniseer::grid;

  namespace
  {

    // DDA march over segment [A,B]; visit each crossed cell once.
    // The visitor returns false to stop the march early.
    template <typename Visitor>
    inline void visit_segment_cells_dda(const GridU8& g, double ax, double ay, double bx, double by,
                                        Visitor&& visit)
    {
      // Convert endpoints to cell coordinates.
      int x = 0, y = 0;
      if (!world_to_cell(g, ax, ay, x, y))
        return;

      // Direction
      const double dx = bx - ax;
      const double dy = by - ay;

      const int stepx = (dx > 0) - (dx < 0);
      const int stepy = (dy > 0) - (dy < 0);

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

      // March forward until we pass boundary or exit the grid.
      double t = 0.0;
      while (in_bounds(x, y, g.width, g.height) && t <= 1.0)
      {
        // Step to next boundary
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

  } // namespace

  int count_unknown_ig(const GridU8& g, const Params& p, const Pose2D& origin,
                       std::vector<std::uint8_t>* scratch_seen)
  {
    auto num_rays    = p.ray_cast_params.num_rays;
    auto max_ray_len = p.ray_cast_params.max_ray_len;

    if (g.width <= 0 || g.height <= 0 || max_ray_len <= 0.0 || num_rays <= 0)
      return 0;

    // Prepare "seen" bitmap to de-duplicate cells across rays.
    const std::size_t n = static_cast<std::size_t>(g.width) * static_cast<std::size_t>(g.height);
    std::vector<std::uint8_t> owned_seen;
    std::uint8_t*             seen_ptr = nullptr;

    if (scratch_seen && scratch_seen->size() == n)
    {
      std::fill(scratch_seen->begin(), scratch_seen->end(), std::uint8_t(0));
      seen_ptr = scratch_seen->data();
    }
    else
    {
      owned_seen.assign(n, 0u);
      seen_ptr = owned_seen.data();
    }

    // Convert origin to cell; if origin is outside, IG is zero.
    int cx0 = 0, cy0 = 0;
    if (!world_to_cell(g, origin.x, origin.y, cx0, cy0))
      return 0;

    // Precompute direction angles.
    const double step_ang = (2.0 * M_PI) / static_cast<double>(num_rays);

    int unknown_total = 0;

    for (int k = 0; k < num_rays; ++k)
    {
      const double th = step_ang * static_cast<double>(k);
      const double ex = origin.x + max_ray_len * std::cos(th);
      const double ey = origin.y + max_ray_len * std::sin(th);

      visit_segment_cells_dda(g, origin.x, origin.y, ex, ey,
                              [&](int x, int y) -> bool
                              {
                                const int     lin = idx(x, y, g.width);
                                const uint8_t v   = g.data[lin];

                                // Count unknowns once across the whole fan.
                                if (is_unknown(v, p))
                                {
                                  if (!seen_ptr[lin])
                                  {
                                    seen_ptr[lin] = 1u;
                                    ++unknown_total;
                                  }
                                  // Continue through unknown space.
                                  return true;
                                }

                                // Stop at any occluder
                                if (is_occluder(v, p))
                                  return false;

                                // Free/traversable cell: continue.
                                return true;
                              });
    }

    return unknown_total;
  }

} // namespace omniseer
