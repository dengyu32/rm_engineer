#pragma once

#include <optional>

#include "step_executor/types/task.hpp"
#include "task_orchestrator/protocol.hpp"

namespace task_orchestrator {

// ============================================================================
//  TaskOrchestrator
// ----------------------------------------------------------------------------
//  - 输入 TaskId，输出 TaskPlan
//  - 负责任务编排（显式步骤）
// ============================================================================

class TaskOrchestrator {
public:
  std::optional<step_executor::TaskPlan> plan(TaskId request) const;
};

} // namespace task_orchestrator
