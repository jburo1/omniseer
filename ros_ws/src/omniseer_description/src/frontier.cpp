#include "omniseer/frontier.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <random>
#include <unordered_map>

#include "omniseer/grid_utils.hpp"

/**
 * \file
 * \brief Implementation of core frontier detection and goal selection algorithms.
 *
 * Implements the pipeline declared in \c frontier.hpp:
 *  - \ref omniseer::compute_frontier_mask
 *  - \ref omniseer::label_components
 *  - \ref omniseer::select_component_goals
 *  - \ref omniseer::compute_goal_information
 *  - \ref omniseer::select_top_k_by_information
 *  - \ref omniseer::score_goals_with_precomputed_cost
 *  - \ref omniseer::rank_goals
 *
 *  - \ref [deprecated] omniseer::find_frontier_goals
 *
 * \ingroup omniseer_frontier
 */

namespace omniseer
{
  namespace
  {
    /**
     * \internal
     * \brief Return neighbor offset table for the chosen connectivity.
     *
     * For 4-connectivity, returns 4 (dx,dy) pairs; for 8-connectivity, returns 8 pairs.
     * \param c Connectivity (Four or Eight).
     * \param[out] count Number of neighbor pairs provided (4 or 8).
     * \return Pointer to static interleaved array: {dx0,dy0, dx1,dy1, ... }.
     */
    const int* neighbor_table(Conn c, int& count) noexcept
    {
      static const int N4[8]  = {1, 0, -1, 0, 0, 1, 0, -1};
      static const int N8[16] = {1, 0, -1, 0, 0, 1, 0, -1, 1, 1, 1, -1, -1, 1, -1, -1};
      if (c == Conn::Four)
      {
        count = 4;
        return N4;
      }
      count = 8;
      return N8;
    }

  } // namespace

  using namespace omniseer::grid;

  void compute_frontier_mask(const GridU8& g, const Params& p, std::uint8_t* out_mask,
                             std::size_t n)
  {
    // Input validation
    if (!out_mask || n != static_cast<std::size_t>(g.width) * static_cast<std::size_t>(g.height))
    {
      return;
    }

    const int  w              = g.width;
    const int  h              = g.height;
    int        neighbor_count = 0;
    const int* neighbors      = neighbor_table(p.connectivity, neighbor_count);

    // Compute mask
    for (int y = 0; y < h; ++y)
    {
      for (int x = 0; x < w; ++x)
      {
        const int     i     = idx(x, y, w);
        const uint8_t value = g.data[i];

        // Frontier = traversable cell with >= min_unknown_neighbors UNKNOWN neighbors
        if (!is_traversable(value, p))
        {
          out_mask[i] = 0;
          continue;
        }

        // Count unknown neighbours
        int unknown = 0;
        for (int k = 0; k < neighbor_count; ++k)
        {
          const int nx = x + neighbors[2 * k];
          const int ny = y + neighbors[2 * k + 1];
          if (!in_bounds(nx, ny, w, h))
            continue;
          const uint8_t nv = g.data[idx(nx, ny, w)];
          if (is_unknown(nv, p))
          {
            if (++unknown >= p.min_unknown_neighbors)
              break;
          }
        }
        out_mask[i] = (unknown >= p.min_unknown_neighbors) ? 1 : 0;
      }
    }
  }

  std::vector<FrontierComponent> label_components(const GridU8& g, const Params& p,
                                                  std::uint8_t* frontier_mask, std::size_t n,
                                                  int* out_component_labels)
  {
    std::vector<FrontierComponent> comps;

    // Input validation
    const int w = g.width;
    const int h = g.height;
    if (!frontier_mask || n != static_cast<std::size_t>(w) * static_cast<std::size_t>(h))
    {
      return comps;
    }

    if (out_component_labels)
    {
      std::fill_n(out_component_labels, n, -1);
    }

    int        neighbor_count = 0;
    const int* neighbors      = neighbor_table(p.connectivity, neighbor_count);

    // Visited bitmap
    std::vector<std::uint8_t> visited(n, 0);

    // Scan all cells and start a DFS for each unvisited frontier pixel
    for (int y = 0; y < h; ++y)
    {
      for (int x = 0; x < w; ++x)
      {
        const int seed = idx(x, y, w);
        if (visited[seed] || frontier_mask[seed] == 0)
          continue;

        std::vector<int> q;
        q.reserve(256);
        q.push_back(seed);
        visited[seed] = 1;

        std::vector<int> members;
        members.reserve(256);

        std::vector<int> rim; // subset of members that touch non-frontier (component boundary)
        rim.reserve(128);

        while (!q.empty())
        {
          const int i = q.back();
          q.pop_back();

          const int cx = i % w;
          const int cy = i / w;

          members.push_back(i);

          bool is_rim = false;

          // Visit neighbors
          for (int k = 0; k < neighbor_count; ++k)
          {
            const int nx = cx + neighbors[2 * k];
            const int ny = cy + neighbors[2 * k + 1];

            if (!in_bounds(nx, ny, w, h))
            {
              // Out of bounds counts as non-frontier neighbor = boundary
              is_rim = true;
              continue;
            }

            const int j = idx(nx, ny, w);
            if (frontier_mask[j] == 0)
            {
              // Neighbor is not frontier = boundary
              if (is_unknown(g.data[j], p))
                is_rim = true;
              // is_rim = true;
            }
            else if (!visited[j])
            {
              visited[j] = 1;
              q.push_back(j);
            }
          }

          if (is_rim)
            rim.push_back(i);
        }

        // Ignore small components
        const int size = static_cast<int>(members.size());
        if (size < p.min_component_size)
        {
          continue;
        }

        const int comp_id = static_cast<int>(comps.size());

        if (out_component_labels)
        {
          for (int idx_lin : members)
          {
            out_component_labels[idx_lin] = comp_id;
          }
        }

        // Build component
        FrontierComponent c;
        c.id         = comp_id;
        c.size_cells = size;

        if (!rim.empty())
        {
          c.rim_indices = std::move(rim);
        }
        else
        {
          c.rim_indices = std::move(members);
        }

        comps.push_back(std::move(c));
      }
    }

    return comps;
  }

  std::vector<FrontierGoal> select_component_goals(const GridU8& g, const Params& p,
                                                   const std::vector<FrontierComponent>& comps)
  {
    std::vector<FrontierGoal> out;

    const int w = g.width, h = g.height;
    if (w <= 0 || h <= 0 || comps.empty())
      return out;

    const int base_spacing    = std::max(1, p.min_goal_spacing_cells);
    const int max_goals_per_c = std::max(1, p.max_goals_per_component);
    const int max_total_goals = std::max(0, p.max_total_goals);

    //  Determine goal share per component
    std::vector<ComponentShare> component_shares;
    component_shares.reserve(comps.size());

    int total_rim_length = 0;
    for (int i = 0; i < (int) comps.size(); ++i)
    {
      const auto& component  = comps[i];
      const int   rim_length = (int) component.rim_indices.size();
      const int   geom_max   = std::min((rim_length + base_spacing - 1) / base_spacing,
                                        max_goals_per_c); // equivalent to ceil(rim_length / base)
      component_shares.push_back(ComponentShare{
          0.0,        // raw_share
          0.0,        // frac
          i,          // component_id
          rim_length, // rim_length
          0,          // goal_budget
          geom_max    // geom_max
      });
      total_rim_length += rim_length;
    }

    // Allocate budgets
    int used = 0;
    for (auto& share : component_shares)
    {
      share.raw_share =
          (double) max_total_goals * (double) share.rim_length / (double) total_rim_length;
      const int base    = (int) std::floor(share.raw_share);
      share.goal_budget = std::min(base, share.geom_max);
      share.frac        = share.raw_share - double(base); // compute remainder
      used += share.goal_budget;
    }
    int left = std::max(0, max_total_goals - used);

    // Allocate leftovers if there are any (Hamilton method)
    if (left > 0)
    {
      std::sort(component_shares.begin(), component_shares.end(),
                [](const ComponentShare& a, const ComponentShare& b)
                {
                  if (a.frac != b.frac)
                    return a.frac > b.frac; // largest remainder first
                  if (a.geom_max - a.goal_budget != b.geom_max - b.goal_budget)
                    return (a.geom_max - a.goal_budget) > (b.geom_max - b.goal_budget);
                  return a.component_id < b.component_id;
                });

      for (auto& share : component_shares)
      {
        if (!left)
          break;
        if (share.goal_budget < share.geom_max)
        {
          ++share.goal_budget;
          --left;
        }
      }
    }

    // Per-component selection with adaptive spacing
    for (const auto& share : component_shares)
    {

      const auto& comp       = comps[share.component_id];
      const int   rim_length = share.rim_length;

      const int s_i    = base_spacing;
      const int s_i_sq = s_i * s_i;

      // Deterministic shuffle per component
      std::vector<int> rim = comp.rim_indices;
      std::mt19937     rng(0x9e3779b9u ^ uint32_t(comp.id) ^ uint32_t(rim_length));
      std::shuffle(rim.begin(), rim.end(), rng);

      BucketGrid buckets(s_i);
      int        accepted = 0;

      for (int idx_lin : rim)
      {
        if (accepted >= share.goal_budget)
          break;

        const int cx = idx_lin % w;
        const int cy = idx_lin / w;
        if (!in_bounds(cx, cy, w, h))
          continue;
        if (!is_traversable(g.data[idx(cx, cy, w)], p))
          continue;
        if (buckets.near_conflict(cx, cy, s_i_sq))
          continue;

        Pose2D pose = cell_center_to_world(cx, cy, g);

        FrontierGoal gk;
        gk.pose         = pose;
        gk.component_id = comp.id;
        out.push_back(gk);

        buckets.insert(cx, cy);
        ++accepted;
      }
    }

    return out;
  }

  void compute_goal_information(const Params& /*p*/, const Callbacks& cb,
                                std::vector<FrontierGoal>& goals)
  {
    const bool have_info = static_cast<bool>(cb.information_gain);
    for (auto& goal : goals)
    {
      double info = 0.0;
      if (have_info)
      {
        info = cb.information_gain(goal.pose);
        if (!std::isfinite(info) || info < 0.0)
          info = 0.0;
      }
      goal.info_gain = info;
    }
  }

  void select_top_k_by_information(std::vector<FrontierGoal>& goals, int top_k)
  {
    if (goals.empty())
      return;

    auto cmp_info = [](const FrontierGoal& A, const FrontierGoal& B)
    {
      if (A.info_gain != B.info_gain)
        return A.info_gain > B.info_gain;
      if (A.component_id != B.component_id)
        return A.component_id < B.component_id;
      if (A.pose.x != B.pose.x)
        return A.pose.x < B.pose.x;
      return A.pose.y < B.pose.y;
    };

    if (top_k > 0 && top_k < static_cast<int>(goals.size()))
    {
      std::nth_element(goals.begin(), goals.begin() + top_k, goals.end(), cmp_info);
      goals.resize(static_cast<size_t>(top_k));
    }

    std::stable_sort(goals.begin(), goals.end(), cmp_info);
  }

  void score_goals_with_precomputed_cost(const Params& p, std::vector<FrontierGoal>& goals)
  {
    if (goals.empty())
      return;

    MinMax mm_info;
    for (const auto& g : goals)
      mm_info.observe(g.info_gain);

    MinMax mm_cost;
    for (const auto& g : goals)
    {
      if (std::isfinite(g.path_cost) && g.path_cost >= 0.0)
        mm_cost.observe(g.path_cost);
    }

    std::vector<FrontierGoal> filtered;
    filtered.reserve(goals.size());

    for (auto& goal : goals)
    {
      if (!std::isfinite(goal.path_cost) || goal.path_cost < 0.0)
        continue;

      const double info_n = mm_info.map(goal.info_gain);
      const double cost_n = mm_cost.map(goal.path_cost);

      double score = (p.w_information * info_n) - (p.w_distance_cost * cost_n);
      if (!std::isfinite(score))
        score = -std::numeric_limits<double>::infinity();
      goal.score = score;

      filtered.push_back(goal);
    }

    auto cmp_goal = [](const FrontierGoal& A, const FrontierGoal& B)
    {
      if (A.score != B.score)
        return A.score > B.score;
      if (A.component_id != B.component_id)
        return A.component_id < B.component_id;
      if (A.pose.x != B.pose.x)
        return A.pose.x < B.pose.x;
      return A.pose.y < B.pose.y;
    };

    std::stable_sort(filtered.begin(), filtered.end(), cmp_goal);
    goals = std::move(filtered);
  }

  void rank_goals(const Params& p, const Pose2D& robot, const Callbacks& cb,
                  std::vector<FrontierGoal>& goals)
  {
    if (goals.empty())
      return;

    compute_goal_information(p, cb, goals);
    select_top_k_by_information(goals, p.top_k_goals);
    if (goals.empty())
      return;

    const bool have_cost = static_cast<bool>(cb.plan_cost);

    std::vector<FrontierGoal> with_cost;
    with_cost.reserve(goals.size());

    for (auto& goal : goals)
    {
      double cost;
      if (have_cost)
      {
        cost = cb.plan_cost(robot, goal.pose);
        if (!std::isfinite(cost) || cost < 0.0)
          continue;
      }
      else
      {
        cost = euclid(robot, goal.pose);
      }
      goal.path_cost = cost;
      with_cost.push_back(goal);
    }

    goals = std::move(with_cost);
    score_goals_with_precomputed_cost(p, goals);
  }

  std::vector<FrontierGoal> find_frontier_goals(const GridU8& g, const Params& p,
                                                const Pose2D& robot, const Callbacks& cb,
                                                Artifacts* artifacts)
  {
    std::vector<FrontierGoal> goals;

    const int    w = static_cast<int>(g.width);
    const int    h = static_cast<int>(g.height);
    const size_t n = static_cast<size_t>(w) * static_cast<size_t>(h);

    // Prepare frontier mask buffer
    std::vector<std::uint8_t> local_mask;
    std::uint8_t*             mask_ptr = nullptr;

    if (artifacts)
    {
      artifacts->width  = w;
      artifacts->height = h;
      artifacts->frontier_mask.assign(n, 0u);
      mask_ptr = artifacts->frontier_mask.data();
    }
    else
    {
      local_mask.assign(n, 0u);
      mask_ptr = local_mask.data();
    }

    // Pipeline
    compute_frontier_mask(g, p, mask_ptr, n);

    const auto comps = label_components(g, p, mask_ptr, n);

    goals = select_component_goals(g, p, comps);

    rank_goals(p, robot, cb, goals);

    return goals;
  }

} // namespace omniseer
