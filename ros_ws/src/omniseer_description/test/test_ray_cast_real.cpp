#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <iomanip>
#include <limits>
#include <numeric>
#include <string>
#include <utility>
#include <vector>

#include "omniseer/frontier.hpp"
#include "omniseer/grid_utils.hpp"
#include "omniseer/ray_cast.hpp"
#include "omniseer_test/test_utils.hpp"

using namespace omniseer;
using namespace omniseer_test;

// Visualize ray-cast IG on the real example costmap for the top-5 ranked goals.
// Produces per-goal overlays and a summary CSV for comparison.
TEST(RayCastReal, VisualizeTop5IGRays)
{
  const std::string root = TEST_DIR;
  GridU8 g = load_pgm_with_meta(root + "/examples/map.pgm", root + "/examples/map.meta.json");

  // Frontier detection params to propose candidate goals
  Params p;
  p.free_cost_max           = 200;
  p.connectivity            = Conn::Eight;
  p.min_unknown_neighbors   = 1;
  p.min_component_size      = 15;
  p.min_goal_spacing_cells  = 6;
  p.max_goals_per_component = 30;
  p.max_total_goals         = 60;

  // IG via ray cast only
  p.w_information               = 1.0;
  p.w_distance_cost             = 0.0;
  p.top_k_goals                 = 5;
  p.ray_cast_params.num_rays    = 128;
  p.ray_cast_params.max_ray_len = 3.0;

  const size_t         N = static_cast<size_t>(g.width) * static_cast<size_t>(g.height);
  std::vector<uint8_t> frontier_mask(N, 0);
  compute_frontier_mask(g, p, frontier_mask.data(), frontier_mask.size());
  auto components = label_components(g, p, frontier_mask.data(), frontier_mask.size());
  ASSERT_FALSE(components.empty());

  auto goals = select_component_goals(g, p, components);
  ASSERT_FALSE(goals.empty());

  Callbacks cb;
  cb.information_gain = [&](const Pose2D& goal) -> double
  { return static_cast<double>(count_unknown_ig(g, p, goal)); };

  compute_goal_information(p, cb, goals);
  select_top_k_by_information(goals, p.top_k_goals);

  for (auto& goal : goals)
    goal.path_cost = 0.0; // IG-only ranking; distance term disabled

  score_goals_with_precomputed_cost(p, goals);
  ASSERT_FALSE(goals.empty());

  const int K = std::min(p.top_k_goals, static_cast<int>(goals.size()));
  for (int i = 1; i < K; ++i)
    EXPECT_GE(goals[i - 1].score, goals[i].score);

  const auto out_dir = artifact_dir();

  // Write summary CSV for comparison
  {
    std::ofstream csv(out_dir / "ig_top5_summary.csv");
    csv << "rank,x,y,ig_unique,rays_contrib,rays_no_contrib\n";
    csv << std::fixed << std::setprecision(6);

    for (int i = 0; i < K; ++i)
    {
      const auto&          gg = goals[i];
      std::vector<uint8_t> rgb;
      rgb_from_gray_bg(g, rgb);

      // Draw rays and collect stats
      const RayStats stats = draw_rays_overlay_for_goal(g, p, gg.pose, rgb);

      // Sanity-check against core IG
      const int ig_core = count_unknown_ig(g, p, gg.pose);
      EXPECT_EQ(stats.ig_unique, ig_core);

      // Mark goal location (yellow disk)
      int  cx = 0, cy = 0;
      bool ok = omniseer::grid::world_to_cell(g, gg.pose.x, gg.pose.y, cx, cy);
      ASSERT_TRUE(ok);
      const int goal_rad = std::max(1, int(std::round(0.20 / g.resolution))); // ~20cm disk
      stamp_disk(rgb, g.width, g.height, cx, cy, goal_rad, 255, 255, 0);

      // Save per-goal overlay
      std::ostringstream name;
      name << "ig_rays_rank" << (i + 1) << ".ppm";
      write_ppm_rgb(out_dir / name.str(), g.width, g.height, rgb);

      csv << (i + 1) << "," << gg.pose.x << "," << gg.pose.y << "," << stats.ig_unique << ","
          << stats.rays_contrib << "," << stats.rays_no_contrib << "\n";
    }
  }
}
