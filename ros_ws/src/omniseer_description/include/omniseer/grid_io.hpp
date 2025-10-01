#pragma once
#include <cstdint>
#include <string>
#include <vector>

/**
 * \file
 * \brief Representation and loading of 8-bit occupancy/cost grids from PGM + JSON metadata.
 *
 * Cost semantics (PGM grayscale values):
 *  - 255 : unknown
 *  - 254/253 : obstacles (lethal / inscribed)
 *  - 1..252 : inflation gradient / traversable costs
 *  - 0 : free space
 *
 * \ingroup omniseer_grid
 */

namespace omniseer
{

  /** \defgroup omniseer_grid Grid I/O
   *  Grid representation and loaders (ROS-agnostic).
   *  @{
   */

  /**
   * \brief Row-major 8-bit occupancy/cost grid.
   *
   * Indices are linearized as \c data[y * width + x].
   * Spatial units are meters; \c resolution is meters-per-cell.
   * The grid’s map-frame origin is at (\c origin_x, \c origin_y) in meters.
   */
  struct GridU8
  {
    uint32_t             width{0}, height{0};          /**< Grid dimensions in cells. */
    float                resolution{0.f};              /**< Cell resolution (meters per cell). */
    float                origin_x{0.f}, origin_y{0.f}; /**< Map-frame origin (meters). */
    std::vector<uint8_t> data; /**< Row-major cost data; size = width * height. */
  };

  /**
   * \brief Load a PGM grid and companion JSON metadata into a \ref GridU8.
   *
   * The PGM supplies the 8-bit cost values; the JSON supplies \c width, \c height,
   * \c resolution, \c origin_x, and \c origin_y. The resulting \c data is row-major.
   *
   * \param[in] pgm_path        Path to the PGM image file.
   * \param[in] meta_json_path  Path to the JSON metadata file.
   * \return Populated \ref GridU8 instance.
   *
   * \pre \c pgm_path and \c meta_json_path refer to readable files describing the same grid.
   * \throws std::runtime_error on I/O or parse errors, or on dimension mismatches.
   */
  GridU8 load_pgm_with_meta(const std::string& pgm_path, const std::string& meta_json_path);

  /** @} */ // end of group omniseer_grid

} // namespace omniseer
