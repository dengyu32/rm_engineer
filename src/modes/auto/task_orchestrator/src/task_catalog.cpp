#include "task_orchestrator/task_catalog.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <filesystem>
#include <algorithm>
#include <vector>

#include <yaml-cpp/yaml.h>

#include <cstdio>
#include <stdexcept>
#include <string>

namespace task_orchestrator::detail {
namespace {

namespace fs = std::filesystem;

constexpr const char *kPresetsRelativePath = "config/presets.yaml";
constexpr const char *kTasksDirectoryRelativePath = "config/tasks";

std::vector<std::string> collectTaskFiles(const fs::path &tasks_dir) {
  if (!fs::exists(tasks_dir) || !fs::is_directory(tasks_dir)) {
    throw std::runtime_error("task config directory missing: " + tasks_dir.string());
  }

  std::vector<std::string> files;
  for (const auto &entry : fs::directory_iterator(tasks_dir)) {
    if (!entry.is_regular_file()) {
      continue;
    }
    if (entry.path().extension() != ".yaml" && entry.path().extension() != ".yml") {
      continue;
    }
    files.push_back(entry.path().string());
  }

  std::sort(files.begin(), files.end());
  if (files.empty()) {
    throw std::runtime_error("no task yaml files found in: " + tasks_dir.string());
  }
  return files;
}

} // namespace

TaskCatalog loadTaskCatalog() {
  TaskCatalog catalog{};

  try {
    const std::string share_dir =
        ament_index_cpp::get_package_share_directory("task_orchestrator");
    const fs::path presets_path = fs::path(share_dir) / kPresetsRelativePath;
    const fs::path tasks_dir = fs::path(share_dir) / kTasksDirectoryRelativePath;

    const YAML::Node presets_root = YAML::LoadFile(presets_path.string());
    const std::vector<std::string> task_files = collectTaskFiles(tasks_dir);

    std::vector<YAML::Node> task_roots;
    task_roots.reserve(task_files.size());
    for (const auto &path : task_files) {
      task_roots.push_back(YAML::LoadFile(path));
    }

    catalog = parseTaskCatalogNodes(
        presets_root, task_roots,
        presets_path.string() + " + " + tasks_dir.string());
  } catch (const std::exception &ex) {
    catalog.loaded = false;
    if (catalog.source_path.empty()) {
      catalog.source_path = std::string(kPresetsRelativePath) + " + " + kTasksDirectoryRelativePath;
    }
    catalog.error = ex.what();
  }

  return catalog;
}

const TaskCatalog &taskCatalog() {
  static const TaskCatalog catalog = loadTaskCatalog();
  return catalog;
}

void reportCatalogErrorOnce(const TaskCatalog &catalog) {
  static bool reported = false;
  if (reported || catalog.loaded) {
    return;
  }
  reported = true;
  std::fprintf(stderr, "[task_orchestrator] failed to load %s: %s\n",
               catalog.source_path.empty() ? "<unknown>" : catalog.source_path.c_str(),
               catalog.error.empty() ? "unknown error" : catalog.error.c_str());
}

} // namespace task_orchestrator::detail
