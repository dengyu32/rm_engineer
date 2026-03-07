#pragma once

#include "task_orchestrator/i_task_orchestrator.hpp"

namespace task_orchestrator {

class StaticTaskOrchestrator : public ITaskOrchestrator {
public:
  std::optional<task_step_library::TaskPlan>
  plan(const task_step_library::TaskRequest &request) const override;
};

} // namespace task_orchestrator
