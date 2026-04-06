#pragma once

#include <array>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "auto_library/method.hpp"
#include "auto_library/task.hpp"
#include "task_orchestrator/protocol.hpp"

namespace task_orchestrator::detail {

using DoubleArray = std::vector<double>;
using DoubleTable = std::vector<std::array<double, 6>>;

inline constexpr std::size_t kTaskCount = static_cast<std::size_t>(TaskId::COUNT);

struct TaskCatalog {
  std::unordered_map<std::string, DoubleArray> double_arrays{};
  std::unordered_map<std::string, std::shared_ptr<DoubleTable>> double_tables{};
  std::array<std::optional<core::TaskPlan>, kTaskCount> plans{};
  std::string source_path{};
  std::string error{};
  bool loaded{false};
};

TaskCatalog parseTaskCatalogFile(const std::string &source_path,
                                 const core::KindSpecMap &kind_specs);
TaskCatalog parseTaskCatalogNodes(const YAML::Node &presets_root,
                                  const std::vector<YAML::Node> &task_roots,
                                  const core::KindSpecMap &kind_specs,
                                  std::string source_path);
TaskCatalog loadTaskCatalog(const core::KindSpecMap &kind_specs);
void reportCatalogErrorOnce(const TaskCatalog &catalog);

} // namespace task_orchestrator::detail
