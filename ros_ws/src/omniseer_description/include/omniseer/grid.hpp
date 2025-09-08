#pragma once
#include <cstdint>
#include <string>
#include <vector>

struct GridU8
{
  uint32_t             width{0}, height{0};
  float                resolution{0.f};
  float                origin_x{0.f}, origin_y{0.f};
  std::vector<uint8_t> data;
};

GridU8 load_pgm_with_meta(const std::string& pgm_path, const std::string& meta_json_path);
