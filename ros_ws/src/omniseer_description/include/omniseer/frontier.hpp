#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "omniseer/grid_io.hpp"
#include "omniseer/ray_cast.hpp"

/**
 * \file
 * \brief ROS-agnostic costmap frontier detection and goal selection API.
 * The BT node governing the robot's goal selection wraps this logic.
 *
 * Given a costmap (GridU8), identify frontier cells, group these into connected components,
 * propose goal poses within these (allocated via component size, selected via random
 * sample with closeness constraint), and rank those goals for exploration based on information gain
 * (2D DDA) and path length/feasability (nav2 planner/euclid), and return the top-k. Optionally,
 * populate an artifacts struct for debugging/testing/visualization purposes.
 *
 * \ingroup omniseer_frontier
 */

namespace omniseer
{
  /**
   * \brief Grid connectivity used for neighborhood queries.
   */
  enum class Conn : uint8_t
  {
    Four  = 4, /**< 4-connected neighborhood. */
    Eight = 8  /**< 8-connected neighborhood. */
  };

  /**
   * \brief 2D pose (no yaw) in the map frame.
   */
  struct Pose2D
  {
    double x{0.0}; /**< X position */
    double y{0.0}; /**< Y position */
  };

  /**
   * \brief Frontier detection and goal-selection parameters.
   *
   * Cost semantics follow:
   * 0 = free, 255 = unknown, 254 = lethal obstacle, 253 = inscribed obstacle.
   */
  struct Params
  {
    RaycastParams ray_cast_params; /**< Parameters for IG 2D DDA raycast */

    double robot_radius_m      = 0.20; /**< Robot radius used for feasibility checks. */
    double goal_clearance_m    = 0.10; /**< Required clearance around goal pose. */
    double max_goal_distance_m = 20.0; /**< Ignore goals beyond this distance from robot. */
    double w_information       = 1.0;  /**< Weight for information gain in scoring. */
    double w_distance_cost     = 1.0;  /**< Weight for distance/planning cost in scoring. */

    int min_component_size      = 10; /**< Discard frontier components smaller than this # cells. */
    int min_unknown_neighbors   = 1;  /**< Minimum unknown neighbors to qualify as frontier. */
    int min_goal_spacing_cells  = 10; /**< Min cell distance between goals within a component. */
    int max_goals_per_component = 30; /**< Max number of goals per component. */
    int max_total_goals         = 30; /**< Max number of goals in total. */
    int top_k_goals             = 10; /**< <=0 => keep all; >0 => return only top-K goals */

    uint8_t free_cost_max  = 150; /**< Maximum cost considered traversable. */
    uint8_t unknown_cost   = 255; /**< Value designating unknown cells. */
    uint8_t lethal_cost    = 254; /**< Value designating lethal obstacles. */
    uint8_t inscribed_cost = 253; /**< Value designating inscribed obstacles. */
    Conn    connectivity =
        Conn::Eight; /**< Neighborhood used when testing unknown neighbors / components. */
  };

  /**
   * \brief Callback hooks for planning cost, information gain, and logging.
   */
  struct Callbacks
  {
    /**
     * \brief Calculate path length to goal.
     * \param robot Current robot pose.
     * \param goal Target goal pose.
     * \return Non-negative cost or -1 if not achievable.
     */
    std::function<double(const Pose2D& robot, const Pose2D& goal)> plan_cost;

    /**
     * \brief Estimate information gain at a goal pose.
     * \param goal Target goal pose.
     * \return Non-negative information value.
     */
    std::function<double(const Pose2D& goal)> information_gain;

    /**
     * \brief Logging hook for diagnostic messages.
     * \param msg Message string.
     */
    std::function<void(const std::string& msg)> log;
  };

  /**
   * \brief Stateful/streaming Min–max - normalize values to [0,1].
   * \details
   * Maintains the smallest (\ref lo) and largest (\ref hi) values observed so far via
   * \ref observe. The \ref map function linearly normalizes an input \p v to the
   * closed unit interval.
   *
   * * \code
   * MinMax mm;
   * for (double x : {3.0, 7.0, 5.0}) mm.observe(x);
   * double y = mm.map(5.0); // y == 0.5
   * \endcode
   */
  struct MinMax
  {
    double lo{std::numeric_limits<double>::infinity()};
    double hi{-std::numeric_limits<double>::infinity()};

    void observe(double v)
    {
      if (std::isfinite(v))
      {
        lo = std::min(lo, v);
        hi = std::max(hi, v);
      }
    }
    double map(double v) const
    {
      // Guard against degenerate or uninitialized ranges
      if (!(std::isfinite(lo) && std::isfinite(hi)) || hi <= lo)
        return 0.0;
      return (v - lo) / (hi - lo);
    }
  };

  /**
   * \brief Connected frontier component summary.
   */
  struct FrontierComponent
  {
    std::vector<int> rim_indices;   /**< Representative frontier cell indices (linear y*w + x). */
    int              id{-1};        /**< Component id in [0, K-1] or -1 if unset. */
    int              size_cells{0}; /**< Number of frontier cells in the component. */
  };

  /**
   * \brief Candidate exploration goal with score.
   */
  struct FrontierGoal
  {
    Pose2D pose;           /**< Goal pose. */
    double info_gain{0.0}; /**< Value of unknown cells within a radius of pose. */
    double path_cost{std::numeric_limits<double>::infinity()}; /**< Value of path from robot's
                                                                  current pose to pose */
    double score{0.0};                                         /**< Higher is better. */
    int    component_id{-1}; /**< Id of originating frontier component. */
  };

  /**
   * \brief Per-component bookkeeping record for goal allocation.
   */
  struct ComponentShare
  {
    double raw_share;    /**< Proportional share of gloval goal budget for this component */
    double frac;         /**< Fractional share of gloval goal budget for this component */
    int    component_id; /**< Corresponding component id */
    int    rim_length;   /**< Rim length of component */
    int    goal_budget;  /**< Goal budget for this component after adjustment */
    int    geom_max; /**< Upper bound on number of goals given geometric and budget constraints */
  };

  /**
   * \brief Bucket grid for spacing (local to each component)
   *
   * Spatial hash of integer cell coords -> small vectors of candidate points.
   */
  struct BucketGrid
  {
    // Width/height of a bucket in cells
    int bucket_size;

    // Hash table: 64-bit key = (bx<<32)|by  -> points in that bucket.
    // Each point is an (x,y) cell coordinate.
    std::unordered_map<uint64_t, std::vector<std::pair<int, int>>> bins;

    explicit BucketGrid(int b_) : bucket_size(std::max(1, b_)) {}

    // Pack two signed 32-bit bucket coords into one 64-bit key.
    static uint64_t key(int bx, int by)
    {
      return (uint64_t(uint32_t(bx)) << 32) | uint32_t(by);
    }

    // Map a cell (cx,cy) to its bucket (bx,by) via integer division.
    inline std::pair<int, int> bucket_of(int cx, int cy) const
    {
      return {cx / bucket_size, cy / bucket_size};
    }

    // Is there an existing point within sqrt(min_sq) cells of (cx,cy)?
    bool near_conflict(int cx, int cy, int min_sq) const
    {
      const auto [bx, by] = bucket_of(cx, cy);

      for (int dy = -1; dy <= 1; ++dy)
        for (int dx = -1; dx <= 1; ++dx)
        {
          auto it = bins.find(key(bx + dx, by + dy));
          if (it == bins.end())
            continue;

          for (const auto& pt : it->second)
          {
            const int dx2 = pt.first - cx;
            const int dy2 = pt.second - cy;
            if (dx2 * dx2 + dy2 * dy2 < min_sq)
              return true;
          }
        }
      return false;
    }

    // Insert a new point into its bucket.
    void insert(int cx, int cy)
    {
      const auto [bx, by] = bucket_of(cx, cy);
      bins[key(bx, by)].push_back({cx, cy});
    }
  };

  /**
   * \brief Optional artifacts produced during the pipeline for debugging/inspection.
   */
  struct Artifacts
  {
    std::vector<FrontierGoal>   candidate_goals;  /**< Candidate goals sampled from components. */
    std::vector<FrontierGoal>   ranked_goals;     /**< Goals after final ranking (descending). */
    std::optional<FrontierGoal> selected_goal;    /**< Best goal selected by the BT node. */
    std::vector<int>            component_labels; /**< Per-cell component id or -1 if none. */
    std::vector<std::uint8_t> frontier_mask; /**< Frontier mask: 0 = not frontier, 1 = frontier. */
    int                       width{0}, height{0}; /**< Grid dimensions for per-cell arrays. */
  };

  // --- Pipeline API ---

  /**
   * \brief Compute the frontier mask: border between traversable (<= free_cost_max) and unknown
   * cells.
   *
   * \param g Input grid.
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
   * \param g Input grid.
   * \param p Parameters.
   * \param[in,out] frontier_mask Buffer of size n where non-zero entries indicate frontier.
   * \param n Size of \p frontier_mask; must equal g.width * g.height.
   * \return Vector of detected \ref FrontierComponent records (id, size, rim indices).
   *
   * \pre n == g.width * g.height
   *
   * \note \p frontier_mask contents are treated as input (0/1); labels returned separately.
   */
  std::vector<FrontierComponent> label_components(const GridU8& g, const Params& p,
                                                  std::uint8_t* frontier_mask, std::size_t n,
                                                  int* out_component_labels = nullptr);

  /**
   * \brief Propose goal poses for each component
   *
   * \param g Input grid.
   * \param p Parameters (radii, clearances, max distance).
   * \param comps Frontier components to seed goal proposals.
   * \return Candidate goals.
   */
  std::vector<FrontierGoal> select_component_goals(const GridU8& g, const Params& p,
                                                   const std::vector<FrontierComponent>& comps);

  /**
   * \brief Score and sort goals in-place (descending by \c score), return top k set in \ref p.
   *
   * \param p Parameters with scoring weights.
   * \param cb Callback bundle: \c plan_cost, \c information_gain.
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
   * \param cb Callbacks.
   * \param artifacts Optional sink for intermediate artifacts (may be nullptr).
   * \return Ranked goals (best first). Empty if no feasible goals found.
   *
   * \note If \p artifacts is non-null, intermediate products are filled for
   * debugging/visualization.
   */
  [[deprecated("Pipeline now split for async computations")]] std::vector<FrontierGoal>
  find_frontier_goals(const GridU8& g, const Params& p, const Pose2D& robot, const Callbacks& cb,
                      Artifacts* artifacts);

  /**
   * \brief Populate the \ref FrontierGoal::info_gain field using the provided callback.
   *
   * \param p Parameters (weights and thresholds; currently used for validation only).
   * \param cb Callback bundle; \c information_gain is optional (treated as zero when absent).
   * \param[in,out] goals Goals whose \c info_gain field will be updated.
   */
  void compute_goal_information(const Params& p, const Callbacks& cb,
                                std::vector<FrontierGoal>& goals);

  /**
   * \brief Keep only the top-k goals by information gain and sort them descending by that value.
   *
   * \param[in,out] goals Goals to filter and reorder in-place.
   * \param top_k Maximum number of goals to keep; <=0 retains the entire set.
   */
  void select_top_k_by_information(std::vector<FrontierGoal>& goals, int top_k);

  /**
   * \brief Score and sort goals assuming information gain and path cost were already computed.
   *
   * \param p Parameters containing scoring weights.
   * \param[in,out] goals Goals to score and sort.
   */
  void score_goals_with_precomputed_cost(const Params& p, std::vector<FrontierGoal>& goals);

} // namespace omniseer
