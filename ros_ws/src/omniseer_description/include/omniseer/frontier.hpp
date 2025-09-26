#pragma once

#include <cstdint>
#include <functional>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include "omniseer/grid_io.hpp"

/*
ROS-agnostic frontier detection and goal selection API test code.

Purpose:
- Given a costmap (GridU8 from grid_io.hpp), identify frontier cells,
  group them into connected components, propose feasible goal poses, and rank those
  goals for exploration based on information gain and distance from current position. Return maximal
  value goal, or -1 if none exist.

  Types:
- Pose2D: {x, y} in map frame
- Conn: 4- or 8-neighborhood for component labeling.
- Params: thresholds, connectivity, size filters, robot radius/clearance, range caps,
          and scoring weights.
*/

namespace omniseer
{

  struct Pose2D
  {
    double x{0.0};
    double y{0.0};
  };

  enum class Conn : uint8_t
  {
    Four  = 4,
    Eight = 8
  };

  struct Params
  {
    // --- Costmap thresholds ---
    uint8_t free_cost_max  = 150; // 0 ..
    uint8_t unknown_cost   = 255; // unkown
    uint8_t lethal_cost    = 254; // lethal obstacle
    uint8_t inscribed_cost = 253; // inscribed obstacle

    // Frontier = cell with at least this many UNKNOWN neighbors.
    Conn connectivity = Conn::Eight;

    int min_component_size    = 10; // cells; discard smaller components
    int min_unknown_neighbors = 1;  // 1..8 depending on connectivity

    // --- Goal generation / validation ---
    double robot_radius_m      = 0.20;
    double goal_clearance_m    = 0.10;
    double max_goal_distance_m = 20.0;

    double w_information   = 1.0;
    double w_distance_cost = 1.0;
  };

  struct Callbacks
  {
    // Return true if goal is achievable from current position
    std::function<bool(const Pose2D& goal, double robot_radius_m, double clearance_m)>
        is_pose_navigable;

    // Planning/path cost from robot to goal
    std::function<double(const Pose2D& robot, const Pose2D& goal)> plan_cost;

    // Information gain estimate from robot to goal
    std::function<double(const Pose2D& goal)> information_gain;

    // Logging hook
    std::function<void(const std::string& msg)> log;
  };

  struct FrontierComponent
  {
    int              id{-1};               // 0..K-1
    int              size_cells{0};        // number of frontier cells
    std::vector<int> rim_indices;          // representative frontier cell indices (y*w + x)
    double           cx_m{0.0}, cy_m{0.0}; // centroid
  };

  struct FrontierGoal
  {
    Pose2D pose;
    double score{0.0};
    int    component_id{-1};
  };

  struct Artifacts
  {
    int                       width{0}, height{0}; // dimensions for per-cell arrays
    std::vector<std::uint8_t> frontier_mask;       // 0/1
    std::vector<std::int32_t> component_labels;    // -1 or component id
    std::vector<float>        distance_field_m;    // >=0 or +inf
    std::vector<float>        goal_heatmap;        // arbitrary finite or NaN

    std::vector<FrontierGoal> goals_pre_rank;
    std::vector<FrontierGoal> goals_ranked;
  };

  // --- Pipeline API ---

  // Compute frontier from costmap, i.e. the border between freeish and unknown
  //   on return: out_mask size = g.width * g.height 0 = not frontier, 1 = frontier
  void compute_frontier_mask(const GridU8& g, const Params& p, std::uint8_t* out_mask,
                             std::size_t n);

  // Compute connected components on the frontier using BFS, store metadata
  //   on entry:  labels[i] = 0 for non-frontier, 1 for frontier (from compute_frontier_mask)
  //   on return: labels[i] = -1 for non-frontier, or component id 0..K-1 for frontier
  std::vector<FrontierComponent> label_components(const GridU8& g, const Params& p,
                                                  std::uint8_t* frontier_mask, std::size_t n);

  // Propose feasible goals per component using callbacks for validity/cost
  std::vector<FrontierGoal> select_component_goals(const GridU8& g, const Params& p,
                                                   const Pose2D& robot, const Callbacks& cb,
                                                   const std::vector<FrontierComponent>& comps);

  // Score and sort goals in-place (descending by score)
  void rank_goals(const Params& p, const Pose2D& robot, std::vector<FrontierGoal>& goals);

  // Full pipeline with optional debug artifacts sink
  std::vector<FrontierGoal> find_frontier_goals(const GridU8& g, const Params& p,
                                                const Pose2D& robot, const Callbacks& cb,
                                                Artifacts* artifacts);

} // namespace omniseer
