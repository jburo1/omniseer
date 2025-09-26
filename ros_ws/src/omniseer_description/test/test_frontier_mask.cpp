#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <string>
#include <vector>

#include "omniseer/frontier.hpp"
#include "omniseer/grid_io.hpp"
#include "omniseer_test/test_utils.hpp"

using namespace omniseer;
using namespace omniseer_test;

/*
This test verifies the compute_frontier_mask functionality on a small GridU8

It checks all the cells and their computed frontier value against what is expected
*/
TEST(FrontierMask, EightConn_UnknownAdjacency)
{
  const uint8_t U = 255, F = 0;
  GridU8        g =
      mk_grid(5, 5, {U, U, U, U, U, U, F, F, F, U, U, F, F, F, U, U, F, F, F, U, U, U, U, U, U});

  Params p;
  p.free_cost_max         = 1;
  p.connectivity          = Conn::Eight;
  p.min_unknown_neighbors = 1;

  std::vector<uint8_t> mask(g.width * g.height);
  compute_frontier_mask(g, p, mask.data(), mask.size());

  auto I = [&](int x, int y) { return y * 5 + x; };

  // Center is NOT frontier
  EXPECT_EQ(mask[I(2, 2)], 0);

  // Cells on the inner rim touching unknown ARE frontier
  EXPECT_EQ(mask[I(1, 1)], 1);
  EXPECT_EQ(mask[I(2, 1)], 1);
  EXPECT_EQ(mask[I(3, 1)], 1);
  EXPECT_EQ(mask[I(1, 3)], 1);
  EXPECT_EQ(mask[I(3, 3)], 1);

  // Unknown cells are never frontier
  EXPECT_EQ(mask[I(0, 0)], 0);
  EXPECT_EQ(mask[I(4, 4)], 0);
}

/*
This test executes compute_frontier_mask() on a real costmap extracted from
simulation

It creates visual artifacts corresponding to free, unknown, frontier, and frontier overlay
representations and stores them in /build/omniseer_description/artifacts/<TestSuite>/<TestName> for
inspection
*/
TEST(FrontierMask, VisualizeFromExamplesMap)
{
  // TEST_DIR is set by CMake to ${CMAKE_SOURCE_DIR}
  const std::string root = TEST_DIR;
  const std::string pgm  = root + "/examples/map.pgm";
  const std::string meta = root + "/examples/map.meta.json";

  GridU8 g = load_pgm_with_meta(pgm, meta);

  Params p;
  p.free_cost_max         = 200;
  p.connectivity          = Conn::Eight;
  p.min_unknown_neighbors = 1;

  std::vector<uint8_t> mask(static_cast<size_t>(g.width) * g.height, 0);
  compute_frontier_mask(g, p, mask.data(), mask.size());

  // Assert non-zero frontier
  size_t cnt = 0;
  for (auto v : mask)
    cnt += (v != 0);
  ASSERT_GT(cnt, 0u) << "No frontier detected—check map encoding and Params.";

  // Write artifacts (pgm dumps) into the test's artifact directory
  std::vector<uint8_t> vis = mask;
  for (auto& v : vis)
    v = v ? 255 : 0;

  const size_t         N = size_t(g.width) * g.height;
  std::vector<uint8_t> free_mask(N), unknown_mask(N), frontier_vis(N);

  for (size_t i = 0; i < N; ++i)
  {
    const uint8_t c = g.data[i];
    free_mask[i]    = (c <= p.free_cost_max && c < p.inscribed_cost) ? 255 : 0;
    unknown_mask[i] = (c == p.unknown_cost) ? 255 : 0;
    frontier_vis[i] = mask[i] ? 255 : 0;
  }

  const std::filesystem::path out = artifact_dir();
  write_pgm((out / "free_mask.pgm").string(), g.width, g.height, free_mask);
  write_pgm((out / "unknown_mask.pgm").string(), g.width, g.height, unknown_mask);
  write_pgm((out / "frontier_mask.pgm").string(), g.width, g.height, frontier_vis);
  write_ppm_overlay((out / "frontier_overlay.ppm").string(), g, mask);
}