#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <iomanip>
#include <limits>
#include <numeric>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include "omniseer/frontier.hpp"
#include "omniseer/grid_io.hpp"
#include "omniseer_test/test_utils.hpp"

using namespace omniseer_test;

using namespace omniseer;

namespace
{

  // Simple SVG doc skeleton
  struct Svg
  {
    std::ostringstream ss;
    int                w, h;
    explicit Svg(int width, int height) : w(width), h(height)
    {
      ss << R"(<?xml version="1.0" encoding="UTF-8"?>)"
         << "\n<svg xmlns='http://www.w3.org/2000/svg' width='" << w << "' height='" << h
         << "' viewBox='0 0 " << w << " " << h << "'>\n"
         << "<style>text{font-family:monospace; font-size:12px} .tick{fill:#444}</style>\n";
    }
    void close()
    {
      ss << "\n</svg>\n";
    }
    void save(const std::filesystem::path& p)
    {
      std::ofstream ofs(p);
      ofs << ss.str();
    }
  };

  // Linear scale helper
  struct Scale
  {
    double d0, d1;
    double r0, r1;
    double operator()(double v) const
    {
      if (!(std::isfinite(v) && std::isfinite(d0) && std::isfinite(d1)) || d1 <= d0)
        return r0;
      const double t = (v - d0) / (d1 - d0);
      return r0 + t * (r1 - r0);
    }
  };

  // MinMax for normalization / axis ranges
  struct MinMax
  {
    double lo = std::numeric_limits<double>::infinity(),
           hi = -std::numeric_limits<double>::infinity();
    void observe(double v)
    {
      if (std::isfinite(v))
      {
        lo = std::min(lo, v);
        hi = std::max(hi, v);
      }
    }
    bool degenerate() const
    {
      return !(std::isfinite(lo) && std::isfinite(hi)) || hi <= lo;
    }
    double map01(double v) const
    {
      return degenerate() ? 0.0 : (v - lo) / (hi - lo);
    }
  };

} // namespace

// --- Minimal goal factory ---
static FrontierGoal mk_goal(double x, double y, int comp_id = 0)
{
  FrontierGoal g;
  g.pose         = Pose2D{x, y};
  g.component_id = comp_id;
  g.score        = -9999.0; // sentinel
  return g;
}

/*
This test verifies:

Failure/edge: Degenerate set
- Empty vector: no crash, no change
*/
TEST(RankGoals, DegenerateInputs_NoCrash)
{
  Params    p; // default weights
  Pose2D    robot{0, 0};
  Callbacks cb; // no callbacks

  std::vector<FrontierGoal> goals;
  rank_goals(p, robot, cb, goals);
  EXPECT_TRUE(goals.empty());

  goals = {mk_goal(5.0, 0.0)};
  rank_goals(p, robot, cb, goals);
  ASSERT_EQ(goals.size(), 1u);
  EXPECT_TRUE(std::isfinite(goals[0].score));
  EXPECT_NEAR(goals[0].score, 0.0, 1e-12);
}

/*
This test verifies:

Distance-only fallback
Nearest goal must rank first (higher score), farthest last.
*/
TEST(RankGoals, DistanceOnlyFallback_SortsByNearness)
{
  Params p;
  p.w_information   = 1.0; // irrelevant (info=0)
  p.w_distance_cost = 1.0;

  Pose2D    robot{0.0, 0.0};
  Callbacks cb; // missing -> distance-only fallback

  std::vector<FrontierGoal> goals{
      mk_goal(10.0, 0.0, 1), // d=10
      mk_goal(1.0, 0.0, 2),  // d=1
      mk_goal(5.0, 0.0, 3),  // d=5
  };

  rank_goals(p, robot, cb, goals);

  ASSERT_EQ(goals.size(), 3u);
  EXPECT_NEAR(goals[0].pose.x, 1.0, 1e-12);
  EXPECT_NEAR(goals[1].pose.x, 5.0, 1e-12);
  EXPECT_NEAR(goals[2].pose.x, 10.0, 1e-12);

  EXPECT_GE(goals[0].score, goals[1].score);
  EXPECT_GE(goals[1].score, goals[2].score);

  for (auto& g : goals)
    EXPECT_NE(g.score, -9999.0);
}

/*
Reads the example map + meta, selects frontier goals, ranks them,
and writes lightweight artifacts for the best 20:
  - rank_goals_top20.ppm          (labels 1..K stamped as fat disks)
  - rank_goals_top20_overlay.ppm  (thick red overlay disks at top-K cells)
  - rank_goals_top20.csv          (rank,x,y,score,component_id,info,cost,info_n,cost_n)
  - rank_goals_top20_bars.svg
  - rank_goals_top20_scatter.svg
*/
TEST(RankGoals, RealMap_Top20Artifacts_BigGoals)
{
  // --- Load map ---
  const std::string root = TEST_DIR;
  GridU8 g = load_pgm_with_meta(root + "/examples/map.pgm", root + "/examples/map.meta.json");

  // --- Frontier detection & components ---
  Params det;
  det.free_cost_max         = 200;
  det.connectivity          = Conn::Eight;
  det.min_unknown_neighbors = 1;
  det.min_component_size    = 15;

  std::vector<uint8_t> mask(size_t(g.width) * g.height, 0);
  compute_frontier_mask(g, det, mask.data(), mask.size());
  auto comps = label_components(g, det, mask.data(), mask.size());
  ASSERT_GT(comps.size(), 0u);

  // --- Candidate goals on rims ---
  Pose2D robot{g.origin_x, g.origin_y};
  auto   goals = select_component_goals(g, det, comps);
  ASSERT_GT(goals.size(), 0u);

  // --- Helpers ---
  auto inb = [&](int x, int y)
  { return x >= 0 && y >= 0 && x < int(g.width) && y < int(g.height); };

  auto to_cell = [&](const Pose2D& p, int& cx, int& cy)
  {
    cx = int(std::floor((p.x - g.origin_x) / g.resolution));
    cy = int(std::floor((p.y - g.origin_y) / g.resolution));
  };

  auto stamp_disk_u8 = [&](std::vector<uint8_t>& img, int cx, int cy, int r, uint8_t val)
  {
    const int r2 = r * r;
    for (int dy = -r; dy <= r; ++dy)
      for (int dx = -r; dx <= r; ++dx)
      {
        if (dx * dx + dy * dy > r2)
          continue;
        const int x = cx + dx, y = cy + dy;
        if (!inb(x, y))
          continue;
        img[I(x, y, g.width)] = val;
      }
  };

  auto stamp_disk_i32 = [&](std::vector<int32_t>& img, int cx, int cy, int r, int32_t val)
  {
    const int r2 = r * r;
    for (int dy = -r; dy <= r; ++dy)
      for (int dx = -r; dx <= r; ++dx)
      {
        if (dx * dx + dy * dy > r2)
          continue;
        const int x = cx + dx, y = cy + dy;
        if (!inb(x, y))
          continue;
        img[I(x, y, g.width)] = val;
      }
  };

  // --- Ranking callbacks ---
  // Tunable IG radius in meters -> cells
  const double ig_radius_m = 1.0; // <-- good default; see notes below
  const int    R_cells     = std::max(1, int(std::round(ig_radius_m / g.resolution)));

  Callbacks cb_rank;
  cb_rank.information_gain = [&](const Pose2D& gp) -> double
  {
    int cx, cy;
    to_cell(gp, cx, cy);
    if (!inb(cx, cy))
      return 0.0;

    int unk = 0;
    for (int dy = -R_cells; dy <= R_cells; ++dy)
      for (int dx = -R_cells; dx <= R_cells; ++dx)
      {
        const int x = cx + dx, y = cy + dy;
        if (!inb(x, y))
          continue;
        // simple circular neighborhood
        if (dx * dx + dy * dy > R_cells * R_cells)
          continue;
        if (g.data[I(x, y, g.width)] == det.unknown_cost)
          ++unk;
      }
    return double(unk);
  };

  cb_rank.plan_cost = [](const Pose2D& r, const Pose2D& gp)
  {
    const double dx = gp.x - r.x, dy = gp.y - r.y;
    return std::sqrt(dx * dx + dy * dy);
  };

  // --- Rank with weights ---
  Params pr;
  pr.w_information   = 1.0;
  pr.w_distance_cost = 1.0;
  rank_goals(pr, robot, cb_rank, goals);
  ASSERT_FALSE(goals.empty());

  // --- Top-K & artifacts (with FAT goal stamps) ---
  const int K = std::min<int>(20, goals.size());
  for (int i = 1; i < K; ++i)
    EXPECT_GE(goals[i - 1].score, goals[i].score); // monotone check

  // Visualization radius in meters -> cells (make goals “bigger”)
  const double vis_radius_m = 0.25; // 25 cm disks read well on 5cm grids
  const int    VIS_R        = std::max(1, int(std::round(vis_radius_m / g.resolution)));

  std::vector<int32_t> label_rank(size_t(g.width) * g.height, -1);
  std::vector<uint8_t> top_mask(size_t(g.width) * g.height, 0);

  for (int i = 0; i < K; ++i)
  {
    const auto& gg = goals[i];
    int         cx, cy;
    to_cell(gg.pose, cx, cy);
    ASSERT_TRUE(inb(cx, cy));

    // sanity: goal location sits on free or <= free_cost_max
    EXPECT_LE(int(g.data[I(cx, cy, g.width)]), int(det.free_cost_max));

    // fat disks for readability
    stamp_disk_i32(label_rank, cx, cy, VIS_R, i + 1); // 1-based rank labels
    stamp_disk_u8(top_mask, cx, cy, VIS_R, 1);        // overlay mask
  }

  const auto out = artifact_dir();
  write_labels_ppm(out / "rank_goals_top20.ppm", g, label_rank);
  write_ppm_overlay(out / "rank_goals_top20_overlay.ppm", g, top_mask);

  {
    // Build normalized info/cost for transparency
    std::vector<double> infos(K), costs(K);
    double              minI = std::numeric_limits<double>::infinity(), maxI = -minI;
    double              minC = std::numeric_limits<double>::infinity(), maxC = -minC;

    for (int i = 0; i < K; ++i)
    {
      infos[i] = cb_rank.information_gain(goals[i].pose);
      costs[i] = cb_rank.plan_cost(robot, goals[i].pose);
      if (std::isfinite(infos[i]))
      {
        minI = std::min(minI, infos[i]);
        maxI = std::max(maxI, infos[i]);
      }
      if (std::isfinite(costs[i]))
      {
        minC = std::min(minC, costs[i]);
        maxC = std::max(maxC, costs[i]);
      }
    }
    auto norm = [](double v, double lo, double hi)
    {
      if (!(std::isfinite(v) && std::isfinite(lo) && std::isfinite(hi)) || hi <= lo)
        return 0.0;
      return (v - lo) / (hi - lo);
    };

    std::ofstream ofs(out / "rank_goals_top20.csv");
    ofs << "rank,x,y,score,component_id,info,cost,info_n,cost_n\n";
    ofs << std::fixed << std::setprecision(6);
    for (int i = 0; i < K; ++i)
    {
      const auto&  gg   = goals[i];
      const double info = infos[i], cost = costs[i];
      ofs << (i + 1) << "," << gg.pose.x << "," << gg.pose.y << "," << gg.score << ","
          << gg.component_id << "," << info << "," << cost << "," << norm(info, minI, maxI) << ","
          << norm(cost, minC, maxC) << "\n";
    }
  }
}

TEST(GoalHelpers, PrecomputedFlowMatchesRankGoals)
{
  Params p;
  p.top_k_goals     = 3;
  p.w_information   = 1.0;
  p.w_distance_cost = 1.0;

  Pose2D robot{0.0, 0.0};

  std::vector<FrontierGoal> goals{
      mk_goal(1.0, 0.0, 0),
      mk_goal(2.0, 0.0, 1),
      mk_goal(3.0, 0.0, 2),
      mk_goal(4.0, 0.0, 3),
  };

  Callbacks cb;
  cb.information_gain = [](const Pose2D& pose) { return 10.0 - pose.x; };
  cb.plan_cost        = [](const Pose2D& start, const Pose2D& goal)
  {
    if (goal.x == 3.0)
      return -1.0;
    return std::hypot(goal.x - start.x, goal.y - start.y);
  };

  auto reference = goals;
  rank_goals(p, robot, cb, reference);

  compute_goal_information(p, cb, goals);
  select_top_k_by_information(goals, p.top_k_goals);

  std::vector<FrontierGoal> staged;
  staged.reserve(goals.size());
  for (auto goal : goals)
  {
    double cost = cb.plan_cost(robot, goal.pose);
    if (!std::isfinite(cost) || cost < 0.0)
      continue;
    goal.path_cost = cost;
    staged.push_back(goal);
  }
  score_goals_with_precomputed_cost(p, staged);

  ASSERT_EQ(staged.size(), reference.size());
  for (size_t i = 0; i < staged.size(); ++i)
  {
    EXPECT_NEAR(staged[i].pose.x, reference[i].pose.x, 1e-12);
    EXPECT_NEAR(staged[i].pose.y, reference[i].pose.y, 1e-12);
    EXPECT_EQ(staged[i].component_id, reference[i].component_id);
    EXPECT_NEAR(staged[i].info_gain, reference[i].info_gain, 1e-12);
    EXPECT_NEAR(staged[i].path_cost, reference[i].path_cost, 1e-12);
    EXPECT_NEAR(staged[i].score, reference[i].score, 1e-12);
  }
}
