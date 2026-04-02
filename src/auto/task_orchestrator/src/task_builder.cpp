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

TaskBuilder &TaskBuilder::vector(const char *label, const geometry_msgs::msg::Vector3 &target,
                                 double target_length_m,
                                 int timeout_ms, int retries) {
  Step step;
  step.type = StepType::ArmMove;
  step.label = label;
  step.timeout_ms = timeout_ms;
  step.max_retries = retries;
  step.arm_move.plan_option = PlanOption::CARTESIAN;
  step.arm_move.vector = target;
  step.arm_move.target_length = target_length_m;
  plan_.steps.push_back(std::move(step));
  return *this;
}

TaskBuilder &TaskBuilder::slot(const char *label, SlotStrategy strategy, int slot_id) {
  Step step;
  step.type = StepType::Slot;
  step.label = label;
  step.slot.strategy = strategy;
  step.slot.slot_id = slot_id;
  plan_.steps.push_back(std::move(step));
  return *this;
}

TaskBuilder &TaskBuilder::slot_mapped_joints(const char *label, int timeout_ms, int retries) {
  Step step;
  step.type = StepType::ArmMove;
  step.label = label;
  step.timeout_ms = timeout_ms;
  step.max_retries = retries;
  step.arm_move.plan_option = PlanOption::JOINTS;
  step.arm_move.joints = std::array<float, 6>{{0.f, 0.f, 0.f, 0.f, 0.f, 0.f}};
  step.arm_move.target_source = TargetSource::SlotMapped;
  plan_.steps.push_back(std::move(step));
  return *this;
}

TaskBuilder &TaskBuilder::vision(const char *label, int timeout_ms, int retries) {
  Step step;
  step.type = StepType::Vision;
  step.label = label;
  step.timeout_ms = timeout_ms;
  step.max_retries = retries;
  plan_.steps.push_back(std::move(step));
  return *this;
}

TaskBuilder &TaskBuilder::vision_mapped_pose(const char *label, int timeout_ms, int retries) {
  Step step;
  step.type = StepType::ArmMove;
  step.label = label;
  step.timeout_ms = timeout_ms;
  step.max_retries = retries;
  step.arm_move.plan_option = PlanOption::NORMAL;
  step.arm_move.target_source = TargetSource::VisionPose;
  plan_.steps.push_back(std::move(step));
  return *this;
}

TaskBuilder &TaskBuilder::vision_mapped_vector(const char *label, double target_length_m,
                                                int timeout_ms, int retries) {
  Step step;
  step.type = StepType::ArmMove;
  step.label = label;
  step.timeout_ms = timeout_ms;
  step.max_retries = retries;
  step.arm_move.plan_option = PlanOption::CARTESIAN;
  step.arm_move.target_source = TargetSource::VisionVector;
  step.arm_move.target_length = target_length_m;
  plan_.steps.push_back(std::move(step));
  return *this;
}

TaskBuilder &TaskBuilder::gripper(const char *label, GripperCommand command) {
  Step step;
  step.type = StepType::Gripper;
  step.label = label;
  step.gripper.command = command;
  plan_.steps.push_back(std::move(step));
  return *this;
}

TaskBuilder &TaskBuilder::delay(const char *label, int delay_ms) {
  Step step;
  step.type = StepType::Delay;
  step.label = label;
  step.delay_ms = delay_ms;
  plan_.steps.push_back(std::move(step));
  return *this;
}

TaskPlan TaskBuilder::build() && { return std::move(plan_); }

TaskPlan TaskBuilder::build() const & { return plan_; }

} // namespace task_orchestrator
