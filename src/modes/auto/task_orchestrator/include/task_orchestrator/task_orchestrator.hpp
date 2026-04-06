#pragma once

#include <optional>

#include "task_orchestrator/task_catalog.hpp"
#include "auto_library/task.hpp"

namespace task_orchestrator {

// ============================================================================
//  TaskOrchestrator
// ----------------------------------------------------------------------------
//  - 输入 TaskId，输出 TaskPlan
//  - 负责任务编排（显式步骤）
// ============================================================================

class TaskOrchestrator {
public:
  explicit TaskOrchestrator(const core::KindSpecMap &kind_specs);

  std::optional<core::TaskPlan> plan(TaskId request) const;

private:
  detail::TaskCatalog catalog_{};
};

} // namespace task_orchestrator
