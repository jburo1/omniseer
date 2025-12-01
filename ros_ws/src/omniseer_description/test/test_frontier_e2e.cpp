#include <algorithm>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <iomanip>
#include <iterator>
#include <sstream>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <vector>

#include "omniseer/frontier.hpp"
#include "omniseer/grid_utils.hpp"
#include "omniseer/ray_cast.hpp"
#include "omniseer_test/test_utils.hpp"

using namespace omniseer;
using namespace omniseer_test;

namespace
{
  // Trim helper for parsing tokens out of the nav2 GetCostmap dump.
  std::string trim_copy(std::string s)
  {
    auto not_space = [](unsigned char ch) { return !std::isspace(ch); };
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), not_space));
    s.erase(std::find_if(s.rbegin(), s.rend(), not_space).base(), s.end());
    return s;
  }

  template <typename T> T parse_scalar(const std::string& buffer, const std::string& key)
  {
    size_t pos = buffer.find(key);
    if (pos == std::string::npos)
      throw std::runtime_error("missing key: " + key);
    pos += key.size();
    size_t end = buffer.find_first_of(",)]", pos);
    if (end == std::string::npos)
      end = buffer.size();
    std::string token = trim_copy(buffer.substr(pos, end - pos));
    if constexpr (std::is_same_v<T, int>)
      return std::stoi(token);
    else
      return static_cast<T>(std::stod(token));
  }

  GridU8 load_costmap_txt(const std::string& path)
  {
    std::ifstream ifs(path);
    if (!ifs.is_open())
      throw std::runtime_error("failed to open costmap: " + path);

    std::string text((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());
    if (text.empty())
      throw std::runtime_error("costmap text file is empty: " + path);

    const int    width      = parse_scalar<int>(text, "size_x=");
    const int    height     = parse_scalar<int>(text, "size_y=");
    const double resolution = parse_scalar<double>(text, "resolution=");

    const std::string origin_prefix =
        "origin=geometry_msgs.msg.Pose(position=geometry_msgs.msg.Point(";
    size_t origin_pos = text.find(origin_prefix);
    if (origin_pos == std::string::npos)
      throw std::runtime_error("origin pose block missing");

    size_t origin_end = text.find("), orientation", origin_pos);
    if (origin_end == std::string::npos)
      throw std::runtime_error("origin pose block truncated");
    std::string origin_block = text.substr(origin_pos, origin_end - origin_pos);

    const double origin_x = parse_scalar<double>(origin_block, "x=");
    const double origin_y = parse_scalar<double>(origin_block, "y=");

    size_t data_pos = text.find("data=[");
    if (data_pos == std::string::npos)
      throw std::runtime_error("data array missing");
    data_pos += 6;
    size_t data_end = text.find(']', data_pos);
    if (data_end == std::string::npos)
      throw std::runtime_error("data array not terminated");
    std::string data_block = text.substr(data_pos, data_end - data_pos);

    std::vector<uint8_t> data;
    data.reserve(static_cast<size_t>(width) * static_cast<size_t>(height));
    std::stringstream ss(data_block);
    std::string       token;
    while (std::getline(ss, token, ','))
    {
      token = trim_copy(token);
      if (token.empty())
        continue;
      const int value = std::stoi(token);
      if (value < 0 || value > 255)
        throw std::runtime_error("costmap cell outside [0,255]: " + token);
      data.push_back(static_cast<uint8_t>(value));
    }

    if (data.size() != static_cast<size_t>(width) * static_cast<size_t>(height))
      throw std::runtime_error("costmap cell count mismatch");

    GridU8 g;
    g.width      = static_cast<uint32_t>(width);
    g.height     = static_cast<uint32_t>(height);
    g.resolution = static_cast<float>(resolution);
    g.origin_x   = static_cast<float>(origin_x);
    g.origin_y   = static_cast<float>(origin_y);
    g.data       = std::move(data);
    return g;
  }

  int pose_to_index(const GridU8& g, const Pose2D& pose)
  {
    if (g.resolution <= 0.0f)
      return -1;
    const double fx = (pose.x - g.origin_x) / static_cast<double>(g.resolution);
    const double fy = (pose.y - g.origin_y) / static_cast<double>(g.resolution);
    const int    cx = static_cast<int>(std::floor(fx));
    const int    cy = static_cast<int>(std::floor(fy));
    if (cx < 0 || cy < 0)
      return -1;
    if (cx >= static_cast<int>(g.width) || cy >= static_cast<int>(g.height))
      return -1;
    return I(cx, cy, static_cast<int>(g.width));
  }
} // namespace

/*
This test verifies:

The recorded nav2 costmap can be parsed into a GridU8.

The frontier pipeline (mask -> components -> goals -> ranking) produces non-empty results.

Visual artifacts for each stage are written to the test artifact directory for manual inspection.
*/
TEST(FrontierE2E, ExampleCostmapPipelineArtifacts)
{
  const std::string costmap_txt = std::string(TEST_DIR) + "/examples/costmap_example.txt";

  GridU8 g;
  ASSERT_NO_THROW(g = load_costmap_txt(costmap_txt));
  ASSERT_EQ(g.data.size(), static_cast<size_t>(g.width) * static_cast<size_t>(g.height));
  ASSERT_GT(g.resolution, 0.0f);

  const size_t N = g.data.size();

  Params p;
  p.free_cost_max           = 200;
  p.connectivity            = Conn::Eight;
  p.min_unknown_neighbors   = 1;
  p.min_component_size      = 30;
  p.w_information           = 1.0;
  p.w_distance_cost         = 1.0;
  p.top_k_goals             = 10;
  p.max_goals_per_component = 30;
  p.max_total_goals         = 60;
  p.min_goal_spacing_cells  = 10;

  Pose2D robot;
  robot.x = g.origin_x + 0.5 * static_cast<double>(g.width) * g.resolution;
  robot.y = g.origin_y + 0.5 * static_cast<double>(g.height) * g.resolution;

  std::vector<uint8_t> frontier_mask(N, 0);
  compute_frontier_mask(g, p, frontier_mask.data(), frontier_mask.size());
  const size_t frontier_count = std::count(frontier_mask.begin(), frontier_mask.end(), uint8_t{1});
  ASSERT_GT(frontier_count, 0u) << "no frontier cells detected";

  auto components = label_components(g, p, frontier_mask.data(), frontier_mask.size());
  ASSERT_FALSE(components.empty()) << "expected at least one frontier component";

  size_t total_cells = 0;
  for (const auto& c : components)
    total_cells += static_cast<size_t>(c.size_cells);
  ASSERT_GT(total_cells, 0u);

  Callbacks cb;
  cb.information_gain = [&, N](const Pose2D& q) -> double
  {
    // One scratch bitmap per thread; avoids realloc & makes parallel loops safe.
    thread_local std::vector<std::uint8_t> scratch;
    if (scratch.size() != N)
      scratch.assign(N, 0);
    return static_cast<double>(count_unknown_ig(g, p, q, &scratch));
  };
  cb.plan_cost = [&](const Pose2D& start, const Pose2D& goal) -> double
  { return omniseer::grid::euclid(start, goal); };

  auto goals_pre_rank = select_component_goals(g, p, components);
  ASSERT_FALSE(goals_pre_rank.empty()) << "no goals sampled from components";

  auto goals_topk = goals_pre_rank;
  compute_goal_information(p, cb, goals_topk);
  select_top_k_by_information(goals_topk, p.top_k_goals);

  if (cb.plan_cost)
  {
    for (auto& goal : goals_topk)
      goal.path_cost = cb.plan_cost(robot, goal.pose);
  }
  else
  {
    for (auto& goal : goals_topk)
      goal.path_cost = omniseer::grid::euclid(robot, goal.pose);
  }

  auto goals_ranked = goals_topk;
  score_goals_with_precomputed_cost(p, goals_ranked);
  ASSERT_FALSE(goals_ranked.empty()) << "goal ranking resulted in an empty set";

  ASSERT_LE(goals_ranked.size(), goals_pre_rank.size());
  if (p.top_k_goals > 0)
  {
    ASSERT_LE(goals_ranked.size(), static_cast<size_t>(p.top_k_goals));
  }
  else
  {
    ASSERT_EQ(goals_ranked.size(), goals_topk.size());
  }
  // Monotone non-increasing scores
  for (size_t i = 1; i < goals_ranked.size(); ++i)
  {
    EXPECT_GE(goals_ranked[i - 1].score, goals_ranked[i].score);
  }

  // --- Artifact generation ---
  const std::filesystem::path out = artifact_dir();

  // Raw costmap as grayscale PGM
  write_pgm(out / "stage0_costmap.pgm", g.width, g.height, g.data);

  // Frontier mask (0/255) and overlay
  std::vector<uint8_t> frontier255 = frontier_mask;
  for (auto& v : frontier255)
    v = v ? 255 : 0;
  write_pgm(out / "stage1_frontier_mask.pgm", g.width, g.height, frontier255);
  write_ppm_overlay(out / "stage1_frontier_overlay.ppm", g, frontier_mask);

  // Component rims, colorized by id, plus overlay mask
  std::vector<int32_t> component_labels(N, -1);
  std::vector<uint8_t> component_rims(N, 0);
  for (const auto& c : components)
  {
    for (int idx : c.rim_indices)
    {
      component_labels[idx] = c.id;
      component_rims[idx]   = 1;
    }
  }
  write_labels_ppm(out / "stage2_components_rims.ppm", g, component_labels);
  write_ppm_overlay(out / "stage2_components_overlay.ppm", g, component_rims);

  auto dump_goal_mask = [&](const std::vector<FrontierGoal>& goals, std::vector<uint8_t>& mask_out,
                            std::vector<int32_t>& labels_out, bool color_by_rank)
  {
    std::fill(mask_out.begin(), mask_out.end(), 0);
    std::fill(labels_out.begin(), labels_out.end(), -1);
    for (size_t i = 0; i < goals.size(); ++i)
    {
      const int idx = pose_to_index(g, goals[i].pose);
      ASSERT_NE(idx, -1) << "goal pose projects out of grid";
      mask_out[static_cast<size_t>(idx)] = 1;
      labels_out[static_cast<size_t>(idx)] =
          color_by_rank ? static_cast<int32_t>(i) : goals[i].component_id;
    }
  };

  std::vector<uint8_t> goal_mask(N, 0);
  std::vector<int32_t> goal_labels(N, -1);

  dump_goal_mask(goals_pre_rank, goal_mask, goal_labels, /*color_by_rank=*/false);
  // Simple red overlay
  write_ppm_overlay(out / "stage3_goals_prerank_overlay.ppm", g, goal_mask);
  write_labels_ppm(out / "stage3_goals_prerank_by_component.ppm", g, goal_labels);

  dump_goal_mask(goals_ranked, goal_mask, goal_labels, /*color_by_rank=*/true);
  // Simple red overlay
  write_ppm_overlay(out / "stage4_goals_ranked_overlay.ppm", g, goal_mask);
  write_rank_gradient_ppm(out / "stage4_goals_ranked_by_order.ppm", g, goal_labels);

  std::ofstream summary(out / "stage4_ranked_scores.txt");
  ASSERT_TRUE(summary.is_open());
  summary << "rank,score,component_id,x_m,y_m\n";
  summary << std::fixed << std::setprecision(3);
  for (size_t i = 0; i < goals_ranked.size(); ++i)
  {
    summary << (i + 1) << ',' << goals_ranked[i].score << ',' << goals_ranked[i].component_id << ','
            << goals_ranked[i].pose.x << ',' << goals_ranked[i].pose.y << '\n';
  }

  // Stage 5: Top-ranked goal IG mask (PGM)
  // Visualize unique-unknown contributions (white) for the best goal,
  // and mark the goal cell mid-gray for reference.
  if (!goals_ranked.empty())
  {
    std::vector<std::uint8_t> seen(N, 0);
    const auto&               best = goals_ranked.front();
    const int                 ig   = count_unknown_ig(g, p, best.pose, &seen);
    EXPECT_GT(ig, 0) << "Top goal IG should be positive";

    std::vector<std::uint8_t> ig_mask(N, 0);
    for (size_t i = 0; i < N; ++i)
      ig_mask[i] = seen[i] ? 255 : 0;

    const int best_idx = pose_to_index(g, best.pose);
    if (best_idx >= 0)
      ig_mask[static_cast<size_t>(best_idx)] = 128; // mark goal cell

    write_pgm(out / "stage5_top_goal_ig_mask.pgm", g.width, g.height, ig_mask);

    // Also produce a color PPM with rays overlay in the same style as the ray-cast test
    {
      std::vector<uint8_t> rgb;
      rgb_from_gray_bg(g, rgb);
      const RayStats stats = draw_rays_overlay_for_goal(g, p, best.pose, rgb);

      // Sanity-check against core IG
      const int ig_core = count_unknown_ig(g, p, best.pose);
      EXPECT_EQ(stats.ig_unique, ig_core);

      // Mark goal location (yellow disk)
      int  cx = 0, cy = 0;
      bool ok = omniseer::grid::world_to_cell(g, best.pose.x, best.pose.y, cx, cy);
      ASSERT_TRUE(ok);
      const int goal_rad = std::max(1, int(std::round(0.20 / g.resolution))); // ~20cm disk
      stamp_disk(rgb, g.width, g.height, cx, cy, goal_rad, 255, 255, 0);

      write_ppm_rgb(out / "stage5_top_goal_ig_rays.ppm", g.width, g.height, rgb);
    }
  }
}
