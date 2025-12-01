#pragma once
/**
 * \file
 * \brief DDA raycasting for calculating a given goal's information gain (IG).
 * In this context, a goal has high IG if it has sight of many unobstructed unknown regions
 * in the costmap within a bounded range.
 *
 * Casts a fan of 2D DDA rays from a pose and counts unique UNKNOWN cells
 * within a radius, stopping each ray on occluders.
 *
 * \ingroup omniseer_frontier
 */

#include <cstdint>
#include <vector>

#include "omniseer/grid_io.hpp"

namespace omniseer
{
  // Forward declare
  struct GridU8;
  struct Pose2D;
  struct Params;

  /**
   * \brief Ray cast parameters.
   */
  struct RaycastParams
  {
    double max_ray_len = 3.0;               /**< Maximum ray length, tune to YOLO detection range*/
    int    num_rays    = 48;                /**< Number of rays */
    double fov_rad     = 6.283185307179586; /**< Y position (2*PI default)*/
    double angle0      = 0.0;               /**< start angle (rad), relative to +x in map frame */
  };
  /**
   * \brief Count unique UNKNOWN cells visible from \p origin within \p radius_m
   *        using a fan of \p num_rays 2D DDA rays.
   * (http://www.cse.yorku.ca/~amana/research/grid.pdf)
   *
   * A ray marches cell-by-cell from the origin and:
   *  - Increments the total once for each DISTINCT cell with value == \ref Params::unknown_cost
   *    (deduplicated across rays).
   *  - Stops when it hits an occluder cell (value > \ref Params::free_cost_max), leaves the
   * grid, or reaches the radius bound.
   *
   * \param g Input grid.
   * \param p Frontier/cost/IG semantics.
   * \param origin Robot position/origin of ray fan.
   * \param scratch_seen Scratch bitmap (size w*h) reused by caller to avoid realloc across multiple
   * goals.
   * \return Count of unique unknown cells seen across all rays.
   */
  int count_unknown_ig(const GridU8& g, const Params& p, const Pose2D& origin,
                       std::vector<std::uint8_t>* scratch_seen = nullptr);
} // namespace omniseer
