#include <algorithm>
#include <cmath>
#include <gtest/gtest.h>
#include <numeric>
#include <string>
#include <unordered_map>
#include <vector>

#include "omniseer/frontier.hpp"
#include "omniseer/grid_io.hpp"
#include "omniseer_test/test_utils.hpp"

using namespace omniseer;
using namespace omniseer_test;

// --- Utilities ---

static std::unordered_map<int, const FrontierComponent*> index_components_by_id(
    const std::vector<FrontierComponent>& comps)
{
  std::unordered_map<int, const FrontierComponent*> m;
  for (const auto& c : comps)
    m[c.id] = &c;
  return m;
}

/*
This test verifies:

We can compute frontiers, label components, then select sample goals
*/
TEST(SelectGoals, RealMap_Artifacts)
{
  // Load the costmap
  std::string root = TEST_DIR;
  GridU8      g = load_pgm_with_meta(root + "/examples/map.pgm", root + "/examples/map.meta.json");

  // Frontier detection params
  Params det;
  det.free_cost_max           = 200;
  det.connectivity            = Conn::Eight;
  det.min_unknown_neighbors   = 1;
  det.min_component_size      = 15;
  det.max_goals_per_component = 6;
  det.max_total_goals         = 30;

  // Frontier mask
  std::vector<uint8_t> mask(size_t(g.width) * g.height, 0);
  compute_frontier_mask(g, det, mask.data(), mask.size());
  size_t frontier_count = 0;
  for (auto v : mask)
    frontier_count += (v != 0);
  ASSERT_GT(frontier_count, 0u);

  // Components
  auto comps = label_components(g, det, mask.data(), mask.size());
  ASSERT_GT(comps.size(), 0u);

  // Select goals
  auto goals = select_component_goals(g, det, comps);
  ASSERT_GT(goals.size(), 0u);

  // Sanity: each goal should map to an in-bounds freeish cell and valid component id
  std::vector<uint8_t>                              goal_mask(size_t(g.width) * g.height, 0);
  std::vector<int32_t>                              goal_labels(size_t(g.width) * g.height, -1);
  std::unordered_map<int, const FrontierComponent*> comp_by_id = index_components_by_id(comps);

  for (const auto& gg : goals)
  {
    ASSERT_TRUE(comp_by_id.count(gg.component_id));

    // Recover integer cell (origin/res used in select_component_goals)
    int cx = static_cast<int>(std::floor((gg.pose.x - g.origin_x) / g.resolution));
    int cy = static_cast<int>(std::floor((gg.pose.y - g.origin_y) / g.resolution));
    ASSERT_GE(cx, 0);
    ASSERT_GE(cy, 0);
    ASSERT_LT(cx, int(g.width));
    ASSERT_LT(cy, int(g.height));

    int idx          = I(cx, cy, g.width);
    goal_mask[idx]   = 1;               // for overlay
    goal_labels[idx] = gg.component_id; // for colorized label image

    // Freeish check
    ASSERT_LE(int(g.data[idx]), int(det.free_cost_max));
  }

  // --- Artifacts ---
  auto out = artifact_dir();

  // 1) Red overlay of GOAL CELLS over the costmap
  write_ppm_overlay(out / "goals_overlay.ppm", g, goal_mask);

  // 2) Colorized image of goal cells by component id (non-goal pixels remain background)
  write_labels_ppm(out / "goals_by_component.ppm", g, goal_labels);
}
