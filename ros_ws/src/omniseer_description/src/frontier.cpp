#include "omniseer/frontier.hpp"

#include <cmath>
#include <limits>
#include <queue> // reserved for future use (e.g., BFS variants)

/**
 * \file
 * \brief Implementation of frontier detection and goal selection.
 *
 * Implements the pipeline declared in \c frontier.hpp:
 *  - \ref omniseer::compute_frontier_mask
 *  - \ref omniseer::label_components
 *  - \ref omniseer::select_component_goals
 *  - \ref omniseer::rank_goals
 *  - \ref omniseer::find_frontier_goals
 *
 * Helper utilities in this file are marked \internal to keep public docs clean.
 * \ingroup omniseer_frontier
 */

namespace omniseer
{
  // --------- Helper functions ----------

  /**
   * \internal
   * \brief Check whether (x,y) lies within [0,w)×[0,h).
   */
  static inline bool in_bounds(int x, int y, int w, int h)
  {
    return (unsigned) x < (unsigned) w && (unsigned) y < (unsigned) h;
  }

  /**
   * \internal
   * \brief Convert (x,y) coordinates to row-major linear index.
   * \return y * w + x
   */
  static inline int idx(int x, int y, int w)
  {
    return y * w + x;
  }

  /**
   * \internal
   * \brief Test whether a cell value equals \ref Params::unknown_cost.
   */
  static inline bool is_unknown(uint8_t v, const Params& p)
  {
    return v == p.unknown_cost;
  }

  /**
   * \internal
   * \brief Test whether a cell value is considered traversable (<= \ref Params::free_cost_max).
   */
  static inline bool is_freeish(uint8_t v, const Params& p)
  {
    return v <= p.free_cost_max;
  }

  /**
   * \internal
   * \brief Return neighbor offset table for the chosen connectivity.
   *
   * For 4-connectivity, returns 4 (dx,dy) pairs; for 8-connectivity, returns 8 pairs.
   * \param c Connectivity (Four or Eight).
   * \param[out] count Number of neighbor pairs provided (4 or 8).
   * \return Pointer to static interleaved array: {dx0,dy0, dx1,dy1, ... }.
   */
  static inline const int* neighbor_table(Conn c, int& count)
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

  // --------- Public API ----------

  /**
   * \copydoc omniseer::compute_frontier_mask
   */
  void compute_frontier_mask(const GridU8& g, const Params& p, std::uint8_t* out_mask,
                             std::size_t n)
  {
    // Input validation
    const int w = static_cast<int>(g.width);
    const int h = static_cast<int>(g.height);
    if (!out_mask || n != static_cast<std::size_t>(w) * static_cast<std::size_t>(h))
    {
      return;
    }

    int        ncnt = 0;
    const int* N    = neighbor_table(p.connectivity, ncnt);

    // Compute mask
    for (int y = 0; y < h; ++y)
    {
      for (int x = 0; x < w; ++x)
      {
        const int     i = idx(x, y, w);
        const uint8_t v = g.data[i];

        // Frontier = FREE-ish cell with >= min_unknown_neighbors UNKNOWN neighbors
        if (!is_freeish(v, p))
        {
          out_mask[i] = 0;
          continue;
        }

        int unknown = 0;
        for (int k = 0; k < ncnt; ++k)
        {
          const int nx = x + N[2 * k];
          const int ny = y + N[2 * k + 1];
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

  /**
   * \copydoc omniseer::label_components
   */
  std::vector<FrontierComponent> label_components(const GridU8& g, const Params& p,
                                                  std::uint8_t* frontier_mask, std::size_t n)
  {
    std::vector<FrontierComponent> comps;

    // Input validation
    const int W = static_cast<int>(g.width);
    const int H = static_cast<int>(g.height);
    if (!frontier_mask || n != static_cast<std::size_t>(W) * static_cast<std::size_t>(H))
    {
      return comps;
    }

    int        ncnt = 0;
    const int* N    = neighbor_table(p.connectivity, ncnt);

    // Visited bitmap
    std::vector<std::uint8_t> visited(n, 0);

    auto I   = [&](int x, int y) -> int { return y * W + x; };
    auto inb = [&](int x, int y) -> bool
    {
      return (static_cast<unsigned>(x) < static_cast<unsigned>(W)) &&
             (static_cast<unsigned>(y) < static_cast<unsigned>(H));
    };

    // Scan all cells and start a DFS for each unvisited frontier pixel
    for (int y = 0; y < H; ++y)
    {
      for (int x = 0; x < W; ++x)
      {
        const int seed = I(x, y);
        if (visited[seed] || frontier_mask[seed] == 0)
          continue;

        // DFS
        std::vector<int> q;
        q.reserve(256);
        q.push_back(seed);
        visited[seed] = 1;

        std::vector<int> members;
        members.reserve(256);

        std::vector<int> rim; // subset of members that touch non-frontier (component boundary)
        rim.reserve(128);

        double sum_xc = 0.0, sum_yc = 0.0; // accumulate cell-center coords (in cell units)

        while (!q.empty())
        {
          const int i = q.back();
          q.pop_back();

          const int cx = i % W;
          const int cy = i / W;

          members.push_back(i);
          sum_xc += (cx + 0.5);
          sum_yc += (cy + 0.5);

          bool is_rim = false;

          // Visit neighbors
          for (int k = 0; k < ncnt; ++k)
          {
            const int nx = cx + N[2 * k];
            const int ny = cy + N[2 * k + 1];

            if (!inb(nx, ny))
            {
              // Out of bounds counts as non-frontier neighbor = boundary
              is_rim = true;
              continue;
            }

            const int j = I(nx, ny);
            if (frontier_mask[j] == 0)
            {
              // Neighbor is not frontier = boundary
              is_rim = true;
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

        // Do not take small components
        const int size = static_cast<int>(members.size());
        if (size < p.min_component_size)
        {
          continue;
        }

        // Build component
        FrontierComponent c;
        c.id         = static_cast<int>(comps.size());
        c.size_cells = size;
        c.cx_m       = g.origin_x + static_cast<double>((sum_xc / size) * g.resolution);
        c.cy_m       = g.origin_y + static_cast<double>((sum_yc / size) * g.resolution);

        // Prefer the boundary subset; guarantee non-empty
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

  /**
   * \copydoc omniseer::select_component_goals
   */
  std::vector<FrontierGoal> select_component_goals(const GridU8& g, const Params& p,
                                                   const Pose2D& robot, const Callbacks& cb,
                                                   const std::vector<FrontierComponent>& comps)
  {
    (void) robot; // not used yet; reserved for future heuristics
    (void) cb;    // not used yet; goal feasibility/cost can be plugged here

    std::vector<FrontierGoal> out;

    // Check input
    const int W = static_cast<int>(g.width);
    const int H = static_cast<int>(g.height);
    if (W <= 0 || H <= 0)
      return out;

    for (const auto& comp : comps)
    {
      if (comp.rim_indices.empty())
        continue;

      const int R = static_cast<int>(comp.rim_indices.size());

      // Calculate how many samples per component based on sqrt(size) to bound work.
      const int target_samples =
          std::clamp<int>(static_cast<int>(std::sqrt(std::max(1, comp.size_cells))), 1, 16);
      const int stride = std::max(1, R / target_samples);

      for (int s = 0; s < R; s += stride)
      {
        const int i  = comp.rim_indices[s];
        const int cx = i % W;
        const int cy = i / W;

        if (!in_bounds(cx, cy, W, H))
          continue;
        if (!is_freeish(g.data[idx(cx, cy, W)], p))
          continue;

        Pose2D goal_pose;
        goal_pose.x = g.origin_x + (static_cast<double>(cx) + 0.5) * g.resolution;
        goal_pose.y = g.origin_y + (static_cast<double>(cy) + 0.5) * g.resolution;

        FrontierGoal gk;
        gk.pose         = goal_pose;
        gk.score        = 0.0; // scored later in rank_goals
        gk.component_id = comp.id;

        out.push_back(gk);
      }
    }

    return out;
  }

  /**
   * \copydoc omniseer::rank_goals
   *
   * \note Stub: intended scoring is \f$ w_{info}\cdot IG - w_{dist}\cdot C \f$,
   * where \c IG = \ref Callbacks::information_gain and \c C = \ref Callbacks::plan_cost.
   */
  void rank_goals(const Params& p, const Pose2D& robot, std::vector<FrontierGoal>& goals)
  {
    (void) p;
    (void) robot;
    (void) goals;
  }

  /**
   * \copydoc omniseer::find_frontier_goals
   *
   * \note Stub: currently initializes \ref Artifacts and returns an empty set.
   */
  std::vector<FrontierGoal> find_frontier_goals(const GridU8& g, const Params& p,
                                                const Pose2D& robot, const Callbacks& cb,
                                                Artifacts* artifacts)
  {
    (void) p;
    (void) robot;
    (void) cb;
    if (artifacts)
    {
      artifacts->width  = static_cast<int>(g.width);
      artifacts->height = static_cast<int>(g.height);
      artifacts->frontier_mask.assign(static_cast<std::size_t>(g.width) * g.height, 0);
    }
    return {};
  }

} // namespace omniseer
