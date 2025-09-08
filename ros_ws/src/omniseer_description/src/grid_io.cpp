#include <cstring>
#include <fstream>
#include <sstream>
#include <stdexcept>

#include "nlohmann/json.hpp"
#include "omniseer/grid.hpp"
using nlohmann::json;

static void read_pgm(const std::string& path, uint32_t& W, uint32_t& H, std::vector<uint8_t>& bytes)
{
  std::ifstream f(path, std::ios::binary);
  if (!f)
    throw std::runtime_error("PGM open failed: " + path);

  std::string magic;
  f >> magic;
  if (magic != "P5")
    throw std::runtime_error("PGM magic != P5");
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
  f.get(); // consume single whitespace after header

  if (w <= 0 || h <= 0)
    throw std::runtime_error("PGM invalid dims");
  W = static_cast<uint32_t>(w);
  H = static_cast<uint32_t>(h);
  bytes.resize(static_cast<size_t>(W) * H);
  f.read(reinterpret_cast<char*>(bytes.data()), bytes.size());
  if (f.gcount() != static_cast<std::streamsize>(bytes.size()))
    throw std::runtime_error("PGM payload short read");
}

GridU8 load_pgm_with_meta(const std::string& pgm_path, const std::string& meta_json_path)
{
  GridU8 g;
  // JSON
  std::ifstream jf(meta_json_path);
  if (!jf)
    throw std::runtime_error("meta open failed: " + meta_json_path);
  json m;
  jf >> m;
  g.width      = m.at("width").get<uint32_t>();
  g.height     = m.at("height").get<uint32_t>();
  g.resolution = m.at("resolution").get<float>();
  g.origin_x   = m.at("origin")[0].get<float>();
  g.origin_y   = m.at("origin")[1].get<float>();

  // PGM
  uint32_t             W = 0, H = 0;
  std::vector<uint8_t> bytes;
  read_pgm(pgm_path, W, H, bytes);
  if (W != g.width || H != g.height)
    throw std::runtime_error("PGM dims don't match JSON dims");

  g.data = std::move(bytes);
  return g;
}
