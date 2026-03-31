#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "auto_library/step.hpp"

namespace core {

// ============================================================================
//  TaskPlan / TaskResult
// ----------------------------------------------------------------------------
//  - TaskPlan: 一组线性 Step
//  - TaskResult: 任务执行汇总
// ============================================================================

using TaskId = uint8_t;

struct TaskPlan {
  TaskId task_id{0};
  std::vector<Step> steps;
};

enum class TaskStatus : uint8_t {
  Success = 0,
  Failure = 1,
  Timeout = 2,
  Canceled = 3,
  Running = 4,
};

struct TaskResult {
  TaskStatus status{TaskStatus::Running};
  std::string message;
};

// 修改原因: 统一 TaskPlan 构造入口，避免调用方重复设置 task_id.
inline TaskPlan makeTaskPlan(TaskId task_id) {
  TaskPlan plan{};
  plan.task_id = task_id;
  return plan;
}

// 修改原因: 统一步骤追加方式，减少调用方样板代码.
inline void addStep(TaskPlan &plan, Step step) {
  plan.steps.push_back(std::move(step));
}

} // namespace core
