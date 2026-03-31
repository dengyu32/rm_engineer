#include "task_orchestrator/task_builder.hpp"

#include <utility>

namespace task_orchestrator {

TaskBuilder::TaskBuilder(task_orchestrator::TaskId task_id) {
  // 修改原因: 统一 TaskPlan 构造入口，避免重复设置字段.
  plan_ = core::makeTaskPlan(static_cast<core::TaskId>(task_id));
}

TaskBuilder &TaskBuilder::add(core::Step step) {
  // 修改原因: 统一步骤追加方式，减少样板代码.
  core::addStep(plan_, std::move(step));
  return *this;
}

core::TaskPlan TaskBuilder::build() && { return std::move(plan_); }

core::TaskPlan TaskBuilder::build() const & { return plan_; }

} // namespace task_orchestrator
