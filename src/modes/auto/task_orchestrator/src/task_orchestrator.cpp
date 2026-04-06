#include "task_orchestrator/task_orchestrator.hpp"

namespace task_orchestrator {

TaskOrchestrator::TaskOrchestrator(const core::KindSpecMap &kind_specs)
    : catalog_(detail::loadTaskCatalog(kind_specs)) {}

std::optional<core::TaskPlan> TaskOrchestrator::plan(TaskId request) const {
  if (!is_supported_task(request) || request == TaskId::IDLE) {
    return std::nullopt;
  }

  if (!catalog_.loaded) {
    detail::reportCatalogErrorOnce(catalog_);
    return std::nullopt;
  }

  const auto &plan = catalog_.plans[static_cast<std::size_t>(request)];
  if (!plan) {
    return std::nullopt;
  }
  return *plan;
}

} // namespace task_orchestrator
