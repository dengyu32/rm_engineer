#pragma once

#include <optional>

#include "task_step_library/types.hpp"

namespace task_orchestrator {

class ITaskOrchestrator {
public:
  virtual ~ITaskOrchestrator() = default;

  virtual std::optional<task_step_library::TaskPlan>
  plan(const task_step_library::TaskRequest &request) const = 0;
};

} // namespace task_orchestrator
