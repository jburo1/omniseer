#include <algorithm>
#include <gtest/gtest.h>
#include <string>
#include <vector>

#include "omniseer/frontier.hpp"
#include "omniseer/grid_io.hpp"
#include "omniseer_test/test_utils.hpp"

using namespace omniseer;
using namespace omniseer_test;

static inline int I(int x, int y, int W)
{
  return y * W + x;
}

/*
This test verifies:

Components labeled for two blobs, correct centroids and sizes
*/
TEST(ComponentLabel, TwoBlobs_EightConn_CentroidAndSizes)
{
  GridU8               g = mk_grid(8, 6); // origin=(0,0), res=1 for easy math
  std::vector<uint8_t> mask(size_t(g.width) * g.height, 0);

  // Blob A: 2x2 at (1..2, 1..2)
  mask[I(1, 1, g.width)] = 1;
  mask[I(2, 1, g.width)] = 1;
  mask[I(1, 2, g.width)] = 1;
  mask[I(2, 2, g.width)] = 1;

  // Blob B: 3x1 line at (5..7, 4)
  mask[I(5, 4, g.width)] = 1;
  mask[I(6, 4, g.width)] = 1;
  mask[I(7, 4, g.width)] = 1;

  Params p;
  p.connectivity       = Conn::Eight;
  p.min_component_size = 1;

  auto comps = label_components(g, p, mask.data(), mask.size());
  ASSERT_EQ(comps.size(), 2u);

  // Sizes (unordered)
  std::array<int, 2> sizes{comps[0].size_cells, comps[1].size_cells};
  std::sort(sizes.begin(), sizes.end());
  EXPECT_EQ(sizes[0], 3);
  EXPECT_EQ(sizes[1], 4);

  // Centroids in meters (cell centers)
  // 2x2 at (1,1),(2,1),(1,2),(2,2) → centroid (2.0, 2.0)
  // 3x1 at (5,4),(6,4),(7,4)       → centroid (6.5, 4.5)
  auto near    = [](double a, double b) { return std::abs(a - b) < 1e-6; };
  bool found22 = false, found6545 = false;
  for (auto& c : comps)
  {
    if (near(c.cx_m, 2.0) && near(c.cy_m, 2.0))
      found22 = true;
    if (near(c.cx_m, 6.5) && near(c.cy_m, 4.5))
      found6545 = true;
  }
  EXPECT_TRUE(found22);
  EXPECT_TRUE(found6545);

  // Artifacts: colorize rims and also dump a mask of all rim pixels
  std::vector<int32_t> rim_labels(size_t(g.width) * g.height, -1);
  std::vector<uint8_t> rim_mask(rim_labels.size(), 0);
  for (const auto& c : comps)
  {
    for (int idx : c.rim_indices)
    {
      rim_labels[idx] = c.id;
      rim_mask[idx]   = 1;
    }
  }

  auto out = artifact_dir();
  write_labels_ppm(out / "two_blobs_components_rims.ppm", g, rim_labels);
  // grayscale rim mask as PGM
  for (auto& v : rim_mask)
    v = v ? 255 : 0;
  write_pgm(out / "two_blobs_rim_mask.pgm", g.width, g.height, rim_mask);
}

/*
This test verifies:

Components labeled for diagonal chain, correct centroids and sizes
*/
TEST(ComponentLabel, DiagonalChain_Connectivity)
{
  GridU8               g = mk_grid(5, 5);
  std::vector<uint8_t> mask(size_t(g.width) * g.height, 0);
  mask[I(1, 1, g.width)] = 1;
  mask[I(2, 2, g.width)] = 1;
  mask[I(3, 3, g.width)] = 1;

  Params p4;
  p4.connectivity       = Conn::Four;
  p4.min_component_size = 1;
  Params p8;
  p8.connectivity       = Conn::Eight;
  p8.min_component_size = 1;

  auto c4 = label_components(g, p4, mask.data(), mask.size());
  auto c8 = label_components(g, p8, mask.data(), mask.size());
  EXPECT_EQ(c4.size(), 3u);
  EXPECT_EQ(c8.size(), 1u);

  // Artifacts: colorized rims for each case
  std::vector<int32_t> L4(size_t(g.width) * g.height, -1), L8(L4.size(), -1);
  for (const auto& c : c4)
    for (int idx : c.rim_indices)
      L4[idx] = c.id;
  for (const auto& c : c8)
    for (int idx : c.rim_indices)
      L8[idx] = c.id;

  auto out = artifact_dir();
  write_labels_ppm(out / "diag_4conn_rims.ppm", g, L4);
  write_labels_ppm(out / "diag_8conn_rims.ppm", g, L8);
}

/*
This test verifies:

Minimum size components axed
*/
TEST(ComponentLabel, MinComponentSize_FiltersSmallIslands)
{
  GridU8               g = mk_grid(6, 4);
  std::vector<uint8_t> mask(size_t(g.width) * g.height, 0);

  // big 2x2 at (1..2,1..2)
  mask[I(1, 1, g.width)] = 1;
  mask[I(2, 1, g.width)] = 1;
  mask[I(1, 2, g.width)] = 1;
  mask[I(2, 2, g.width)] = 1;
  // two singletons
  mask[I(4, 1, g.width)] = 1;
  mask[I(5, 3, g.width)] = 1;

  Params p;
  p.connectivity       = Conn::Eight;
  p.min_component_size = 3;

  auto comps = label_components(g, p, mask.data(), mask.size());
  ASSERT_EQ(comps.size(), 1u);
  EXPECT_EQ(comps[0].size_cells, 4);

  // Artifact
  std::vector<int32_t> L(size_t(g.width) * g.height, -1);
  for (const auto& c : comps)
    for (int idx : c.rim_indices)
      L[idx] = c.id;
  write_labels_ppm(artifact_dir() / "filtered_components_rims.ppm", g, L);
}

/*
This test verifies:

Functionality on a real costmap
*/
TEST(ComponentLabel, RealMap_RimsOverlayAndComponents)
{
  // Load the costmap
  std::string root = TEST_DIR;
  GridU8      g = load_pgm_with_meta(root + "/examples/map.pgm", root + "/examples/map.meta.json");

  // Frontier params for detection (permissive for a continuous rim)
  Params det;
  det.free_cost_max         = 200;
  det.connectivity          = Conn::Eight;
  det.min_unknown_neighbors = 1;

  // Compute frontier mask
  std::vector<uint8_t> mask(size_t(g.width) * g.height, 0);
  compute_frontier_mask(g, det, mask.data(), mask.size());
  size_t frontier_count = 0;
  for (auto v : mask)
    frontier_count += (v != 0);
  ASSERT_GT(frontier_count, 0u);

  // Label components (connectivity/min size taken from Params)
  Params lab             = det;
  lab.min_component_size = 15;
  auto comps             = label_components(g, lab, mask.data(), mask.size());
  ASSERT_GT(comps.size(), 0u);

  // Build labels image from rims for visualization
  std::vector<int32_t> L(size_t(g.width) * g.height, -1);
  std::vector<uint8_t> rims01(L.size(), 0);
  for (const auto& c : comps)
  {
    for (int idx : c.rim_indices)
    {
      L[idx]      = c.id;
      rims01[idx] = 1;
    }
  }

  auto out = artifact_dir();
  // Colorized components-by-id (rims only)
  write_labels_ppm(out / "components_rims.ppm", g, L);

  // Frontier rim overlay (red on the costmap)
  write_ppm_overlay(out / "rims_overlay.ppm", g, rims01);

  // Also dump the raw frontier mask (0/255) for reference
  std::vector<uint8_t> frontier255 = mask;
  for (auto& v : frontier255)
    v = v ? 255 : 0;
  write_pgm(out / "frontier_mask.pgm", g.width, g.height, frontier255);
}
