#include "omniseer/vision/class_list.hpp"

#include <fstream>
#include <stdexcept>
#include <utility>

namespace
{
  std::string trim_ascii(std::string text)
  {
    const size_t start = text.find_first_not_of(" \t\r\n\v\f");
    if (start == std::string::npos)
      return {};
    const size_t end = text.find_last_not_of(" \t\r\n\v\f");
    return text.substr(start, end - start + 1);
  }
} // namespace

namespace omniseer::vision
{
  std::vector<std::string> load_class_list_file(const std::string& path)
  {
    std::ifstream ifs(path);
    if (!ifs)
      throw std::runtime_error("failed to open class list: " + path);

    std::vector<std::string> class_names{};
    std::string              line{};
    while (std::getline(ifs, line))
    {
      line = trim_ascii(std::move(line));
      if (line.empty() || line[0] == '#')
        continue;
      class_names.push_back(line);
    }

    if (class_names.empty())
      throw std::runtime_error("class list is empty: " + path);
    return class_names;
  }
} // namespace omniseer::vision
