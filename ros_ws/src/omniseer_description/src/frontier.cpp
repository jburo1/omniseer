#include "omniseer/frontier.hpp"

#include <cmath>
#include <limits>
#include <queue>

namespace omniseer
{
  // Check if x,y is inbounds of w,h grid
  static inline bool in_bounds(int x, int y, int w, int h)
  {
    return (unsigned) x < (unsigned) w && (unsigned) y < (unsigned) h;
  }

  // Given x,y in grid, return corresponding index in 1D container
  static inline int idx(int x, int y, int w)
  {
    return y * w + x;
  }

  // Given value of costmap entry, return if the grid location is unknown
  static inline bool is_unknown(uint8_t v, const Params& p)
  {
    return v == p.unknown_cost;
  }

  // Given value of costmap entry, return if the grid location is freeish
  static inline bool is_freeish(uint8_t v, const Params& p)
  {
    return v <= p.free_cost_max;
  }

  // Neighborhood offsets for 4/8 connectivity (dx,dy pairs)
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

  // --------- public API ----------

  void compute_frontier_mask(const GridU8& g, const Params& p, std::uint8_t* out_mask,
                             std::size_t n)
  {

    // Check input
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

  std::vector<FrontierComponent> label_components(const GridU8& g, const Params& p,
                                                  std::uint8_t* frontier_mask, std::size_t n)
  {
    std::vector<FrontierComponent> comps;

    // Check input
    const int W = static_cast<int>(g.width);
    const int H = static_cast<int>(g.height);
    if (!frontier_mask || n != static_cast<std::size_t>(W) * static_cast<std::size_t>(H))
    {
      return comps;
    }

    // Neighbor table
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

    // Scan all cells and start a BFS for each unvisited frontier pixel
    for (int y = 0; y < H; ++y)
    {
      for (int x = 0; x < W; ++x)
      {
        const int seed = I(x, y);
        if (visited[seed] || frontier_mask[seed] == 0)
          continue;

        // BFS
        std::vector<int> q;
        q.reserve(256);
        q.push_back(seed);
        visited[seed] = 1;

        std::vector<int> members;
        members.reserve(256);

        std::vector<int> rim; // subset of members that touch non-frontier (component boundary)
        rim.reserve(128);

        double sum_xc = 0.0, sum_yc = 0.0; // accumulation of cell-center coords (in cell units)

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

  std::vector<FrontierGoal> select_component_goals(const GridU8& g, const Params& p,
                                                   const Pose2D& robot, const Callbacks& cb,
                                                   const std::vector<FrontierComponent>& comps)
  {
    (void) g;
    (void) p;
    (void) robot;
    (void) cb;
    (void) comps;
    return {};
  }

  void rank_goals(const Params& p, const Pose2D& robot, std::vector<FrontierGoal>& goals)
  {
    (void) p;
    (void) robot;
    (void) goals;
  }

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
