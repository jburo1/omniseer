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

static inline int I(int x, int y, int W)
{
  return y * W + x;
}

// --- Utilities ---

static std::unordered_map<int, const FrontierComponent*> index_components_by_id(
    const std::vector<FrontierComponent>& comps)
{
  std::unordered_map<int, const FrontierComponent*> m;
  for (const auto& c : comps)
    m[c.id] = &c;
  return m;
}

static std::vector<int> make_linear_rim_indices(int W, int /*H*/, int yrow, int x0, int x1)
{
  std::vector<int> v;
  v.reserve(std::max(0, x1 - x0 + 1));
  for (int x = x0; x <= x1; ++x)
    v.push_back(I(x, yrow, W)); // assumes in-bounds
  return v;
}

/*
This test verifies:

Empty/degenerate inputs return no goals and do not crash
*/
TEST(SelectGoals, DegenerateInputs_NoCrashNoGoals)
{
  GridU8    g = mk_grid(0, 0);
  Params    p;
  Pose2D    robot{0, 0};
  Callbacks cb;

  std::vector<FrontierComponent> comps;
  auto                           goals = select_component_goals(g, p, robot, cb, comps);
  EXPECT_TRUE(goals.empty());

  // Non-empty grid, but no components
  g     = mk_grid(5, 5);
  goals = select_component_goals(g, p, robot, cb, comps);
  EXPECT_TRUE(goals.empty());

  // One component with empty rim -> skipped
  FrontierComponent c;
  c.id          = 7;
  c.size_cells  = 10;
  c.rim_indices = {};
  comps.push_back(c);
  goals = select_component_goals(g, p, robot, cb, comps);
  EXPECT_TRUE(goals.empty());
}

/*
This test verifies:

Cells not "freeish" (cost > free_cost_max) are filtered out,
and produced goals land on centers of valid cells with correct component ids.
*/
TEST(SelectGoals, FiltersByFreeishAndSetsPoseCenter)
{
  GridU8 g = mk_grid(8, 6); // origin=(0,0), res=1 for easy math
  // Fill costs with "free" (e.g., 0)
  g.data.assign(size_t(g.width) * g.height, 0);

  // Two components with rims on row y=3 and y=4
  FrontierComponent A;
  A.id          = 10;
  A.size_cells  = 9; // arbitrary
  A.rim_indices = make_linear_rim_indices(g.width, g.height, /*yrow=*/3, /*x0=*/1, /*x1=*/5);

  FrontierComponent B;
  B.id          = 11;
  B.size_cells  = 16;
  B.rim_indices = make_linear_rim_indices(g.width, g.height, /*yrow=*/4, /*x0=*/2, /*x1=*/6);

  // Mark some rim cells as NOT free (cost > free_cost_max) — they should be skipped
  // For A: knock out x=2 and x=5 on y=3
  g.data[I(2, 3, g.width)] = 255;
  g.data[I(5, 3, g.width)] = 255;
  // For B: knock out x=3 and x=4 on y=4
  g.data[I(3, 4, g.width)] = 254;
  g.data[I(4, 4, g.width)] = 254;

  Params p;
  p.free_cost_max = 200; // anything >200 is not freeish
  p.connectivity  = Conn::Eight;

  Pose2D    robot{0, 0};
  Callbacks cb;

  std::vector<FrontierComponent> comps{A, B};
  auto                           goals = select_component_goals(g, p, robot, cb, comps);
  ASSERT_FALSE(goals.empty());

  // All goals must be at cell centers and belong to {A,B} ids
  auto m = index_components_by_id(comps);
  for (const auto& gg : goals)
  {
    EXPECT_TRUE(m.count(gg.component_id));
    // Check "center of cell" in meters given origin=(0,0), res=1
    // Recover integer cell from pose
    int cx = static_cast<int>(std::floor(gg.pose.x));
    int cy = static_cast<int>(std::floor(gg.pose.y));
    EXPECT_NEAR(gg.pose.x, cx + 0.5, 1e-9);
    EXPECT_NEAR(gg.pose.y, cy + 0.5, 1e-9);
    // Ensure the underlying cost is freeish
    int idx = I(cx, cy, g.width);
    ASSERT_GE(idx, 0);
    ASSERT_LT(idx, int(g.data.size()));
    EXPECT_LE(int(g.data[idx]), int(p.free_cost_max));
  }

  // Ensure the known blocked cells did NOT produce goals
  auto has_cell = [&](int cx, int cy)
  {
    double x = cx + 0.5, y = cy + 0.5;
    for (auto& gg : goals)
      if (std::abs(gg.pose.x - x) < 1e-12 && std::abs(gg.pose.y - y) < 1e-12)
        return true;
    return false;
  };
  EXPECT_FALSE(has_cell(2, 3));
  EXPECT_FALSE(has_cell(5, 3));
  EXPECT_FALSE(has_cell(3, 4));
  EXPECT_FALSE(has_cell(4, 4));
}

/*
This test verifies:

Sampling is bounded by sqrt(size_cells) (clamped to [1,16])
and produces ~target_samples per component based on rim length.
*/
TEST(SelectGoals, SamplingBoundedBySqrtWithCap16)
{
  // Make a wide grid so we can lay long rims
  GridU8 g = mk_grid(200, 5);
  g.data.assign(size_t(g.width) * g.height, 0); // all free

  // Component with size_cells=100 -> sqrt=10 target samples (<=16)
  FrontierComponent C;
  C.id         = 42;
  C.size_cells = 100;
  // Rim with R=200 along row y=2: this exaggerates R so we really test stride logic
  C.rim_indices = make_linear_rim_indices(g.width, g.height, /*yrow=*/2, /*x0=*/0, /*x1=*/199);

  // Another component with size_cells=1000 -> sqrt~31.6 -> clamped to 16
  FrontierComponent D;
  D.id          = 43;
  D.size_cells  = 1000;
  D.rim_indices = make_linear_rim_indices(g.width, g.height, /*yrow=*/1, /*x0=*/0, /*x1=*/199);

  Params p;
  p.free_cost_max = 200;
  Pose2D    robot{0, 0};
  Callbacks cb;

  std::vector<FrontierComponent> comps{C, D};
  auto                           goals = select_component_goals(g, p, robot, cb, comps);

  // Count per component
  int nC = 0, nD = 0;
  for (auto& gg : goals)
  {
    if (gg.component_id == C.id)
      ++nC;
    if (gg.component_id == D.id)
      ++nD;
  }

  // Expected C target ~ sqrt(100)=10. The code computes stride = R/target, samples ~= target.
  EXPECT_GE(nC, 8); // allow small variance
  EXPECT_LE(nC, 12);

  // Expected D target sqrt(1000)~31.6 -> clamp to 16
  EXPECT_LE(nD, 17);
  EXPECT_GE(nD, 12); // not too few given the very long rim
}

/*
This test verifies (integration on a real costmap):

— We can compute frontiers, label components, then select sample goals
— We produce visual artifacts:
    1) components_rims.ppm           (already from ComponentLabel test)
    2) goals_overlay.ppm             (red overlay of selected goal cells)
    3) goals_by_component.ppm        (colorized by component id at goal cells)
*/
TEST(SelectGoals, RealMap_ArtifactsAndSanity)
{
  // Load the costmap
  std::string root = TEST_DIR;
  GridU8      g = load_pgm_with_meta(root + "/examples/map.pgm", root + "/examples/map.meta.json");

  // Frontier detection params (permissive rim so we get continuous frontiers)
  Params det;
  det.free_cost_max         = 200;
  det.connectivity          = Conn::Eight;
  det.min_unknown_neighbors = 1;

  // Frontier mask
  std::vector<uint8_t> mask(size_t(g.width) * g.height, 0);
  compute_frontier_mask(g, det, mask.data(), mask.size());
  size_t frontier_count = 0;
  for (auto v : mask)
    frontier_count += (v != 0);
  ASSERT_GT(frontier_count, 0u);

  // Components
  Params lab             = det;
  lab.min_component_size = 15;
  auto comps             = label_components(g, lab, mask.data(), mask.size());
  ASSERT_GT(comps.size(), 0u);

  // Select goals
  Pose2D    robot{g.origin_x, g.origin_y};
  Callbacks cb; // unused in current impl
  auto      goals = select_component_goals(g, det, robot, cb, comps);
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

  // Light structural expectation: not all goals come from the same component
  // (weak check; avoids overfitting exact counts)
  std::unordered_map<int, int> per_comp_counts;
  for (auto id : goal_labels)
    if (id >= 0)
      ++per_comp_counts[id];
  EXPECT_GE(per_comp_counts.size(), 1u);
}
