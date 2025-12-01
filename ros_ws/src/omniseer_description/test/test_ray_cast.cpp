#include <algorithm>
#include <cmath>
#include <cstdint>
#include <gtest/gtest.h>
#include <vector>

#include "omniseer/grid_utils.hpp"
#include "omniseer/ray_cast.hpp"
#include "omniseer_test/test_utils.hpp"

using namespace omniseer;
using namespace omniseer_test;

namespace
{
  inline void set_cell(GridU8& g, int x, int y, uint8_t v)
  {
    ASSERT_GE(x, 0);
    ASSERT_GE(y, 0);
    ASSERT_LT(x, g.width);
    ASSERT_LT(y, g.height);
    g.data[static_cast<size_t>(y) * static_cast<size_t>(g.width) + static_cast<size_t>(x)] = v;
  }

  inline Pose2D cell_center(const GridU8& g, int cx, int cy)
  {
    return omniseer::grid::cell_center_to_world(cx, cy, g);
  }
} // namespace

// No unknowns anywhere => IG is zero
TEST(RayCast, NoUnknown_ReturnsZero)
{
  GridU8 g = mk_grid(7, 7);
  Params p;
  p.free_cost_max               = 200;
  p.ray_cast_params.num_rays    = 64;
  p.ray_cast_params.max_ray_len = 5.0;

  const Pose2D origin = cell_center(g, 3, 3);
  const int    ig     = count_unknown_ig(g, p, origin);
  EXPECT_EQ(ig, 0);
}

// Origin out of bounds => IG is zero
TEST(RayCast, OriginOutOfBounds_ReturnsZero)
{
  GridU8 g = mk_grid(5, 5);
  Params p;
  p.ray_cast_params.num_rays    = 32;
  p.ray_cast_params.max_ray_len = 4.0;

  Pose2D origin{-10.0, 2.0};
  EXPECT_EQ(count_unknown_ig(g, p, origin), 0);
}

// A single unknown cell intersected by many rays counts once (dedupe across rays)
TEST(RayCast, DedupAcrossRays_OneUnknownCountsOnce)
{
  GridU8 g = mk_grid(12, 7);
  Params p;
  p.free_cost_max               = 200;
  p.ray_cast_params.num_rays    = 128;  // plenty of rays crossing the same cell
  p.ray_cast_params.max_ray_len = 20.0; // long enough to reach it

  // Place a lone unknown to the right of origin
  set_cell(g, 9, 3, p.unknown_cost);

  const Pose2D origin = cell_center(g, 2, 3);
  const int    ig     = count_unknown_ig(g, p, origin);
  EXPECT_EQ(ig, 1) << "should count a single unique unknown cell";
}

// Introducing an occluder between origin and an unknown should not increase IG
TEST(RayCast, OccluderBlocksOrEquals)
{
  GridU8 g_open = mk_grid(12, 7);
  GridU8 g_blk  = g_open; // copy

  Params p;
  p.free_cost_max               = 200;
  p.ray_cast_params.num_rays    = 96;
  p.ray_cast_params.max_ray_len = 20.0;

  // Unknown lies to the right; a horizontal ray hits it.
  set_cell(g_open, 6, 3, p.unknown_cost);
  set_cell(g_blk, 6, 3, p.unknown_cost);

  // Occluder one step to the right of the origin blocks nearly-horizontal rays.
  constexpr uint8_t LETHAL = 254;
  set_cell(g_blk, 3, 3, LETHAL);

  const Pose2D origin = cell_center(g_open, 2, 3);
  const int    ig0    = count_unknown_ig(g_open, p, origin);
  const int    ig1    = count_unknown_ig(g_blk, p, origin);

  EXPECT_GE(ig0, ig1);
}

// Increasing max_ray_len should be monotonic non-decreasing (cannot lose unseen unknowns)
TEST(RayCast, MonotoneWithRadius)
{
  GridU8 g = mk_grid(20, 7);
  Params p;
  p.free_cost_max            = 200;
  p.ray_cast_params.num_rays = 64;

  // Put a few unknowns at increasing distances along +x
  set_cell(g, 5, 3, p.unknown_cost);
  set_cell(g, 8, 3, p.unknown_cost);
  set_cell(g, 12, 3, p.unknown_cost);

  const Pose2D origin = cell_center(g, 2, 3);

  p.ray_cast_params.max_ray_len = 1.0; // cannot reach any
  const int ig1                 = count_unknown_ig(g, p, origin);
  p.ray_cast_params.max_ray_len = 3.0; // reach first
  const int ig2                 = count_unknown_ig(g, p, origin);
  p.ray_cast_params.max_ray_len = 6.0; // reach second
  const int ig3                 = count_unknown_ig(g, p, origin);
  p.ray_cast_params.max_ray_len = 15.0; // reach third
  const int ig4                 = count_unknown_ig(g, p, origin);

  EXPECT_LE(ig1, ig2);
  EXPECT_LE(ig2, ig3);
  EXPECT_LE(ig3, ig4);
}
