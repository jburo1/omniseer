#pragma once

#include <cstdint>
#include <functional>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include "omniseer/grid_io.hpp"

/**
 * \file
 * \brief ROS-agnostic costmap frontier detection and goal selection API.
 *
 * Given an occupancy/cost grid (GridU8), identify frontier cells,
 * group them into connected components, propose feasible goal poses using callbacks,
 * and rank those goals for exploration based on information gain and distance.
 *
 * \ingroup omniseer_frontier
 */

namespace omniseer
{

  /**
   * \brief 2D pose in the map frame (meters).
   */
  struct Pose2D
  {
    double x{0.0}; /**< X position */
    double y{0.0}; /**< Y position */
  };

  /**
   * \brief Grid connectivity used for neighborhood queries.
   */
  enum class Conn : uint8_t
  {
    Four  = 4, /**< 4-connected neighborhood. */
    Eight = 8  /**< 8-connected neighborhood. */
  };

  /**
   * \brief Frontier detection and goal-selection parameters.
   *
   * Cost semantics follow:
   *  - 0 = free, 255 = unknown, 254 = lethal obstacle, 253 = inscribed obstacle.
   */
  struct Params
  {
    uint8_t free_cost_max  = 150; /**< Maximum cost considered traversable. */
    uint8_t unknown_cost   = 255; /**< Value designating unknown cells. */
    uint8_t lethal_cost    = 254; /**< Value designating lethal obstacles. */
    uint8_t inscribed_cost = 253; /**< Value designating inscribed obstacles (near-lethal). */

    Conn connectivity =
        Conn::Eight; /**< Neighborhood used when testing unknown neighbors / components. */

    int min_component_size    = 10; /**< Discard frontier components smaller than this # cells. */
    int min_unknown_neighbors = 1;  /**< Minimum unknown neighbors to qualify as frontier (1..8). */

    double robot_radius_m      = 0.20; /**< Robot radius (m) used for feasibility checks. */
    double goal_clearance_m    = 0.10; /**< Required clearance (m) around goal pose. */
    double max_goal_distance_m = 20.0; /**< Ignore goals beyond this distance (m) from robot. */

    double w_information   = 1.0; /**< Weight for information gain in scoring. */
    double w_distance_cost = 1.0; /**< Weight for distance/planning cost in scoring. */
  };

  /**
   * \brief Callback hooks for feasibility, planning cost, information gain, and logging.
   */
  struct Callbacks
  {
    /**
     * \brief Return true if the goal pose is navigable with given radius/clearance.
     * \param goal Goal pose to validate.
     * \param robot_radius_m Robot radius in meters.
     * \param clearance_m Required clearance in meters.
     */
    std::function<bool(const Pose2D& goal, double robot_radius_m, double clearance_m)>
        is_pose_navigable;

    /**
     * \brief Estimate planning/path cost from robot to goal.
     * \param robot Current robot pose.
     * \param goal Target goal pose.
     * \return Non-negative cost (lower is better).
     */
    std::function<double(const Pose2D& robot, const Pose2D& goal)> plan_cost;

    /**
     * \brief Estimate information gain at a goal pose.
     * \param goal Target goal pose.
     * \return Non-negative information value (higher is better).
     */
    std::function<double(const Pose2D& goal)> information_gain;

    /**
     * \brief Logging hook for diagnostic messages.
     * \param msg Message string.
     */
    std::function<void(const std::string& msg)> log;
  };

  /**
   * \brief Connected frontier component summary.
   */
  struct FrontierComponent
  {
    int              id{-1};        /**< Component id in [0, K-1] or -1 if unset. */
    int              size_cells{0}; /**< Number of frontier cells in the component. */
    std::vector<int> rim_indices;   /**< Representative frontier cell indices (linear y*w + x). */
    double           cx_m{0.0}, cy_m{0.0}; /**< Component centroid (meters, map frame). */
  };

  /**
   * \brief Candidate exploration goal with score.
   */
  struct FrontierGoal
  {
    Pose2D pose;             /**< Goal pose (meters). */
    double score{0.0};       /**< Higher is better. */
    int    component_id{-1}; /**< Id of originating frontier component. */
  };

  /**
   * \brief Optional artifacts produced during the pipeline for debugging/inspection.
   */
  struct Artifacts
  {
    int                       width{0}, height{0}; /**< Grid dimensions for per-cell arrays. */
    std::vector<std::uint8_t> frontier_mask; /**< Frontier mask: 0 = not frontier, 1 = frontier. */
    std::vector<std::int32_t> component_labels; /**< Per-cell component id or -1 if none. */
    std::vector<FrontierGoal> goals_pre_rank;   /**< Unsorted candidate goals before scoring. */
    std::vector<FrontierGoal> goals_ranked;     /**< Goals after ranking (descending by score). */
  };

  // --- Pipeline API ---

  /**
   * \brief Compute the frontier mask: border between traversable (<= free_cost_max) and unknown
   * cells.
   *
   * \param g Input grid (dimensions and cost semantics as described in Params).
   * \param p Parameters controlling thresholds and connectivity.
   * \param[out] out_mask Output buffer of size n holding 0 (not frontier) or 1 (frontier).
   * \param n Size of \p out_mask; must equal g.width * g.height.
   *
   * \pre out_mask != nullptr
   * \pre n == g.width * g.height
   */
  void compute_frontier_mask(const GridU8& g, const Params& p, std::uint8_t* out_mask,
                             std::size_t n);

  /**
   * \brief Label connected components over the frontier mask.
   *
   * \param g Input grid (used for bounds and metrics).
   * \param p Parameters (connectivity, min_component_size).
   * \param[in,out] frontier_mask Buffer of size n where non-zero entries indicate frontier.
   * \param n Size of \p frontier_mask; must equal g.width * g.height.
   * \return Vector of detected \ref FrontierComponent records (id, size, rim indices, centroid).
   *
   * \pre frontier_mask != nullptr
   * \pre n == g.width * g.height
   *
   * \note \p frontier_mask contents are treated as input (0/1); labels returned separately.
   */
  std::vector<FrontierComponent> label_components(const GridU8& g, const Params& p,
                                                  std::uint8_t* frontier_mask, std::size_t n);

  /**
   * \brief Propose feasible goal poses for each component using user callbacks.
   *
   * \param g Input grid.
   * \param p Parameters (radii, clearances, max distance).
   * \param robot Current robot pose.
   * \param cb Callback bundle: feasibility (\c is_pose_navigable), \c plan_cost, \c
   * information_gain.
   * \param comps Frontier components to seed goal proposals.
   * \return Candidate goals.
   */
  std::vector<FrontierGoal> select_component_goals(const GridU8& g, const Params& p,
                                                   const Pose2D& robot, const Callbacks& cb,
                                                   const std::vector<FrontierComponent>& comps);

  /**
   * \brief Score and sort goals in-place (descending by \c score).
   *
   * \param p Parameters with scoring weights.
   * \param robot Current robot pose.
   * \param cb Callback bundle: feasibility (\c is_pose_navigable), \c plan_cost, \c
   * information_gain.
   * \param[in,out] goals Goals to score and sort.
   *
   * \post \p goals is sorted in descending order of \c score.
   */
  void rank_goals(const Params& p, const Pose2D& robot, const Callbacks& cb,
                  std::vector<FrontierGoal>& goals);

  /**
   * \brief Full pipeline: frontier detection -> components -> goal proposals -> ranking -> sorting.
   *
   * \param g Input grid.
   * \param p Parameters.
   * \param robot Current robot pose.
   * \param cb Callbacks for feasibility, costs, and info gain.
   * \param artifacts Optional sink for intermediate artifacts (may be nullptr).
   * \return Ranked goals (best first). Empty if no feasible goals found.
   *
   * \note If \p artifacts is non-null, intermediate products are filled for
   * debugging/visualization.
   */
  std::vector<FrontierGoal> find_frontier_goals(const GridU8& g, const Params& p,
                                                const Pose2D& robot, const Callbacks& cb,
                                                Artifacts* artifacts);

} // namespace omniseer
