#include <cmath>
#include <gtest/gtest.h>

#include "omniseer/grid.hpp"
/*
This test verifies:

The PGM and JSON load successfully.

Dimensions are non-zero and consistent.

The data buffer is properly filled.

The origin and resolution produce sane world coordinates.
*/
TEST(GridIo, LoadPgmMeta)
{
  const std::string pgm  = TEST_DIR "/examples/map.pgm";
  const std::string meta = TEST_DIR "/examples/map.meta.json";
  GridU8            g    = load_pgm_with_meta(pgm, meta);

  // PGM / JSON consistency
  ASSERT_GT(g.width, 0u);
  ASSERT_GT(g.height, 0u);
  ASSERT_EQ(g.data.size(), static_cast<size_t>(g.width) * g.height);

  // Spot-check first and last byte exist
  (void) g.data.front();
  (void) g.data.back();

  // World mapping sanity: cell (0,0) center
  const double cx = g.origin_x + 0.5 * g.resolution;
  const double cy = g.origin_y + 0.5 * g.resolution;
  ASSERT_TRUE(std::isfinite(cx));
  ASSERT_TRUE(std::isfinite(cy));
}
