#include "omniseer/grid_io.hpp"

#include <cstring>
#include <fstream>
#include <sstream>
#include <stdexcept>

#include "nlohmann/json.hpp"
using nlohmann::json;

/**
 * \file
 * \brief Load 8-bit occupancy/cost grids from a binary PGM plus JSON metadata.
 *
 * Implements a small loader pair:
 *  - \ref omniseer::read_pgm : parse a binary PGM (P5) into bytes.
 *  - \ref omniseer::load_pgm_with_meta : parse JSON metadata, validate against the PGM, and
 *    construct \ref omniseer::GridU8.
 *
 * JSON is expected in the common ROS map_server style:
 * \code{.json}
 * { "width": W, "height": H, "resolution": m_per_cell, "origin": [ox, oy] }
 * \endcode
 *
 * PGM must be binary P5 with maxval 255. The data are interpreted row-major.
 *
 * \ingroup omniseer_grid
 */

namespace omniseer
{

  /**
   * \brief Read a binary PGM (P5) file into a byte buffer.
   *
   * Validates magic (P5), skips comment lines, enforces \c maxval == 255,
   * reads \c width and \c height, then reads exactly \c width * \c height bytes.
   *
   * \param[in]  path   Filesystem path to the PGM.
   * \param[out] W      Parsed image width (cells).
   * \param[out] H      Parsed image height (cells).
   * \param[out] bytes  Output buffer resized to \c W*H and filled with pixel data.
   *
   * \throws std::runtime_error on open failure, format errors, invalid dimensions,
   *         or short reads.
   *
   * \note The buffer is row-major; index as \c bytes[y * W + x].
   */
  static void read_pgm(const std::string& path, int& W, int& H, std::vector<uint8_t>& bytes)
  {
    std::ifstream f(path, std::ios::binary);
    if (!f)
      throw std::runtime_error("PGM open failed: " + path);

    std::string magic;
    f >> magic;
    if (magic != "P5")
      throw std::runtime_error("PGM magic != P5");

    // Skip PGM comment lines (starting with '#') that may appear after the magic/whitespace.
    auto skip_comments = [&](std::istream& is)
    {
      int c = is.peek();
      while (c == '#')
      {
        std::string line;
        std::getline(is, line);
        c = is.peek();
      }
    };

    skip_comments(f);
    int w = 0, h = 0, maxv = 0;
    f >> w >> h;
    skip_comments(f);
    f >> maxv;
    if (maxv != 255)
      throw std::runtime_error("PGM maxval != 255");
    f.get(); // consume single whitespace byte before payload

    if (w <= 0 || h <= 0)
      throw std::runtime_error("PGM invalid dims");
    W = w;
    H = h;

    bytes.resize(static_cast<std::size_t>(W) * static_cast<std::size_t>(H));
    f.read(reinterpret_cast<char*>(bytes.data()), bytes.size());
    if (f.gcount() != static_cast<std::streamsize>(bytes.size()))
      throw std::runtime_error("PGM payload short read");
  }

  /**
   * \brief Load a PGM grid and companion JSON metadata into a \ref GridU8.
   *
   * The JSON supplies geometry (\c width, \c height, \c resolution, \c origin_x, \c origin_y).
   * The PGM supplies 8-bit cell costs. Dimensions between JSON and PGM are validated.
   *
   * \param[in] pgm_path        Path to the PGM image file.
   * \param[in] meta_json_path  Path to the JSON metadata file.
   * \return Populated \ref GridU8 with row-major data.
   *
   * \throws std::runtime_error on JSON/PGM open or parse failures, or when PGM dimensions
   *         do not match the JSON.
   */
  GridU8 load_pgm_with_meta(const std::string& pgm_path, const std::string& meta_json_path)
  {
    GridU8 g;

    // --- JSON ---
    std::ifstream jf(meta_json_path);
    if (!jf)
      throw std::runtime_error("meta open failed: " + meta_json_path);

    json m;
    jf >> m;
    g.width      = m.at("width").get<int>();
    g.height     = m.at("height").get<int>();
    g.resolution = m.at("resolution").get<float>();
    g.origin_x   = m.at("origin")[0].get<float>();
    g.origin_y   = m.at("origin")[1].get<float>();

    // --- PGM ---
    int                  W = 0, H = 0;
    std::vector<uint8_t> bytes;
    read_pgm(pgm_path, W, H, bytes);
    if (W != g.width || H != g.height)
      throw std::runtime_error("PGM dims don't match JSON dims");

    g.data = std::move(bytes);
    return g;
  }

} // namespace omniseer
