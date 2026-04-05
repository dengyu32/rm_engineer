#include "task_orchestrator/task_orchestrator.hpp"
#include "task_orchestrator/task_catalog.hpp"

namespace task_orchestrator {

std::optional<core::TaskPlan> TaskOrchestrator::plan(TaskId request) const {
  if (!is_supported_task(request) || request == TaskId::IDLE) {
    return std::nullopt;
  }

  const detail::TaskCatalog &catalog = detail::taskCatalog();
  if (!catalog.loaded) {
    detail::reportCatalogErrorOnce(catalog);
    return std::nullopt;
  }

  const auto &plan = catalog.plans[static_cast<std::size_t>(request)];
  if (!plan) {
    return std::nullopt;
  }
  return *plan;
}

} // namespace task_orchestrator
