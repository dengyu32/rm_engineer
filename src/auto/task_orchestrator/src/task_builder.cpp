#include "task_orchestrator/task_builder.hpp"

#include <utility>

namespace task_orchestrator {

TaskBuilder::TaskBuilder(task_orchestrator::TaskId task_id) {
  plan_.task_id = static_cast<step_executor::TaskId>(task_id);
}

TaskBuilder &TaskBuilder::add(step_executor::Step step) {
  plan_.steps.push_back(std::move(step));
  return *this;
}

step_executor::TaskPlan TaskBuilder::build() && { return std::move(plan_); }

step_executor::TaskPlan TaskBuilder::build() const & { return plan_; }

} // namespace task_orchestrator
