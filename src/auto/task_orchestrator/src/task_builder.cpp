#include "task_orchestrator/task_builder.hpp"

#include <utility>

namespace task_orchestrator {
using namespace task_step_library;

TaskBuilder::TaskBuilder(TaskId task_id) { plan_.task_id = task_id; }

TaskBuilder &TaskBuilder::pose(const char *label, const engineer_interfaces::msg::Pose &target,
                               int timeout_ms, int retries) {
  Step step;
  step.type = StepType::ArmMove;
  step.label = label;
  step.timeout_ms = timeout_ms;
  step.max_retries = retries;
  step.arm_move.plan_option = PlanOption::NORMAL;
  step.arm_move.pose = target;
  plan_.steps.push_back(std::move(step));
  return *this;
}

TaskBuilder &TaskBuilder::joints(const char *label, const std::array<float, 6> &target,
                                 int timeout_ms, int retries) {
  Step step;
  step.type = StepType::ArmMove;
  step.label = label;
  step.timeout_ms = timeout_ms;
  step.max_retries = retries;
  step.arm_move.plan_option = PlanOption::JOINTS;
  step.arm_move.joints = target;
  plan_.steps.push_back(std::move(step));
  return *this;
}

TaskBuilder &TaskBuilder::vector(const char *label, const geometry_msgs::msg::Vector3 &target,
                                 int timeout_ms, int retries) {
  Step step;
  step.type = StepType::ArmMove;
  step.label = label;
  step.timeout_ms = timeout_ms;
  step.max_retries = retries;
  step.arm_move.plan_option = PlanOption::CARTESIAN;
  step.arm_move.vector = target;
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

TaskBuilder &TaskBuilder::vision_mapped_vector(const char *label, int timeout_ms, int retries) {
  Step step;
  step.type = StepType::ArmMove;
  step.label = label;
  step.timeout_ms = timeout_ms;
  step.max_retries = retries;
  step.arm_move.plan_option = PlanOption::CARTESIAN;
  step.arm_move.target_source = TargetSource::VisionVector;
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
