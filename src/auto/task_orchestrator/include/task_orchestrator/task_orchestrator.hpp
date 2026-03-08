#pragma once

#include <optional>

#include "task_step_library/task.hpp"

namespace task_orchestrator {

class TaskOrchestrator {
public:
  std::optional<task_step_library::TaskPlan>
  plan(const task_step_library::TaskRequest &request) const;
};

} // namespace task_orchestrator
