#pragma once

#include <optional>

#include "auto_library/task.hpp"
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
  std::optional<core::TaskPlan> plan(TaskId request) const;
};

} // namespace task_orchestrator
