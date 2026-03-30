#pragma once

#include "auto_library/step.hpp"
#include "auto_library/task.hpp"
#include "task_orchestrator/protocol.hpp"

namespace task_orchestrator {

// ============================================================================
//  TaskBuilder
// ----------------------------------------------------------------------------
//  - 便捷构建 TaskPlan
//  - 只做聚合与顺序拼接
// ============================================================================

class TaskBuilder {
public:
  explicit TaskBuilder(TaskId task_id);

  TaskBuilder &add(step_executor::Step step);

  step_executor::TaskPlan build() &&;
  step_executor::TaskPlan build() const &;

private:
  step_executor::TaskPlan plan_{};
};

} // namespace task_orchestrator
