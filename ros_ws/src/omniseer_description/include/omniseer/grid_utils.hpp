#pragma once
/**
 * \file
 * \brief Grid utilities.
 *
 * \ingroup omniseer_frontier
 */

#include <cmath>
#include <cstdint>

#include "omniseer/frontier.hpp"
#include "omniseer/grid_io.hpp"

namespace omniseer::grid
{
  /**
   * \internal
   * \brief Return Euclidean distance between two poses.
   *
   * \param a First pose.
   * \param b Second pose.
   * \return Distance \f$\sqrt{(a_x-b_x)^2 + (a_y-b_y)^2}\f$ in meters.
   */
  inline double euclid(const Pose2D& a, const Pose2D& b) noexcept
  {
    const double dx = a.x - b.x, dy = a.y - b.y;
    return std::sqrt(dx * dx + dy * dy);
  }

  /**
   * \internal
   * \brief Convert (x,y) coordinates to row-major linear index.
   *
   * \param x Cell x-index in [0, w).
   * \param y Cell y-index in [0, h).
   * \param w Grid width in cells.
   * \return Linear index \c y*w + x.
   */
  constexpr int idx(int x, int y, int w) noexcept
  {
    return y * w + x;
  }

  /**
   * \internal
   * \brief Check whether (x,y) lies within [0,w)×[0,h).
   *
   * \param x Cell x-index.
   * \param y Cell y-index.
   * \param w Grid width in cells.
   * \param h Grid height in cells.
   * \return \c true if inside bounds, \c false otherwise.
   */
  constexpr bool in_bounds(int x, int y, int w, int h) noexcept
  {
    return (x >= 0) && (y >= 0) && (x < w) && (y < h);
  }

  /**
   * \internal
   * \brief Test whether a cell value equals \ref Params::unknown_cost.
   *
   * \param v Cell value.
   * \param p Parameters (cost semantics).
   * \return \c true if \p v denotes unknown, else \c false.
   */
  constexpr bool is_unknown(uint8_t v, const Params& p) noexcept
  {
    return v == p.unknown_cost;
  }

  /**
   * \internal
   * \brief Test whether a cell value is considered traversable (<= \ref Params::free_cost_max).
   *
   * \param v Cell value.
   * \param p Parameters (cost semantics).
   * \return \c true if traversable, else \c false.
   */
  constexpr bool is_traversable(uint8_t v, const Params& p) noexcept
  {
    return v <= p.free_cost_max;
  }

  /**
   * \internal
   * \brief Test whether a cell should occlude visibility.
   *
   * \details Any value strictly greater than \ref Params::free_cost_max is treated
   *          as an occluder (includes inscribed/lethal/inflated). Unknown cells
   *          (\ref Params::unknown_cost) are \em not occluders.
   *
   * \param v Cell value.
   * \param p Parameters (cost semantics).
   * \return \c true if the cell occludes visibility, else \c false.
   */
  constexpr bool is_occluder(uint8_t v, const Params& p) noexcept
  {
    return (v != p.unknown_cost) && (v > p.free_cost_max);
  }

  /**
   * \internal
   * \brief Convert center of cell coordinates (cx,cy) to world pose.
   *
   * \param cx Cell x-index.
   * \param cy Cell y-index.
   * \param g Grid specification (origin/resolution).
   * \return Pose at the center of the cell in meters.
   */
  inline Pose2D cell_center_to_world(int cx, int cy, const GridU8& g) noexcept
  {
    Pose2D p;
    p.x = g.origin_x + (static_cast<double>(cx) + 0.5) * g.resolution;
    p.y = g.origin_y + (static_cast<double>(cy) + 0.5) * g.resolution;
    return p;
  }

  /**
   * \internal
   * \brief Convert world coordinates (meters) to cell indices.
   *
   * \details Uses floor-division relative to \c g.origin_{x,y}. The point maps to
   *          the cell whose \em area contains it (no rounding to centers).
   *
   * \param g Grid specification (origin/resolution/dimensions).
   * \param x World x (meters).
   * \param y World y (meters).
   * \param[out] cx Cell x-index.
   * \param[out] cy Cell y-index.
   * \return \c true if the mapped cell lies inside the grid bounds, else \c false.
   *
   * \pre \c g.resolution > 0
   */
  inline bool world_to_cell(const GridU8& g, double x, double y, int& cx, int& cy) noexcept
  {
    cx = static_cast<int>(std::floor((x - g.origin_x) / g.resolution));
    cy = static_cast<int>(std::floor((y - g.origin_y) / g.resolution));
    return in_bounds(cx, cy, g.width, g.height);
  }
} // namespace omniseer::grid
