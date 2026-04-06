#pragma once

#include <string>
#include <unordered_map>
#include <vector>

namespace core {

struct KindSpec {
  std::string kind{};
  std::vector<std::string> allowed_params{};
  std::vector<std::string> required_params{};
  std::vector<std::string> required_outputs{};
};

using KindSpecMap = std::unordered_map<std::string, KindSpec>;

} // namespace core
