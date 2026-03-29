#pragma once

#include <optional>

#include "step_executor/types/task.hpp"
#include "task_orchestrator/protocol.hpp"

namespace task_orchestrator {

class TaskOrchestrator {
public:
  std::optional<step_executor::TaskPlan> plan(TaskId request) const;
};

} // namespace task_orchestrator
