#pragma once

#include <array>

#include <engineer_interfaces/msg/pose.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include "task_step_library/step.hpp"
#include "task_step_library/task.hpp"

namespace task_orchestrator {

constexpr task_step_library::GripperCommand OPEN = task_step_library::GripperCommand::OPEN;
constexpr task_step_library::GripperCommand CLOSE = task_step_library::GripperCommand::CLOSE;

constexpr task_step_library::SlotStrategy PUT = task_step_library::SlotStrategy::SelectSlotToPut;
constexpr task_step_library::SlotStrategy LOCK = task_step_library::SlotStrategy::LockSlot;
constexpr task_step_library::SlotStrategy TAKE = task_step_library::SlotStrategy::SelectSlotToTake;
constexpr task_step_library::SlotStrategy UNLOCK = task_step_library::SlotStrategy::UnlockSlot;

class TaskBuilder {
public:
  explicit TaskBuilder(task_step_library::TaskId task_id);

  TaskBuilder &pose(const char *label, const engineer_interfaces::msg::Pose &target, 
                    int timeout_ms = 8000, int retries = 1);

  TaskBuilder &joints(const char *label, const std::array<float, 6> &target,
                      int timeout_ms = 8000, int retries = 1);

  TaskBuilder &vector(const char *label, const geometry_msgs::msg::Vector3 &target,
                      int timeout_ms = 8000, int retries = 1);

  TaskBuilder &slot(const char *label, task_step_library::SlotStrategy strategy,
                    int slot_id = -1);

  TaskBuilder &slot_mapped_joints(const char *label, int timeout_ms = 8000, int retries = 1);

  TaskBuilder &vision(const char *label, int timeout_ms = 3000, int retries = 0);

  TaskBuilder &vision_mapped_pose(const char *label, int timeout_ms = 8000, int retries = 1);

  TaskBuilder &vision_mapped_vector(const char *label, int timeout_ms = 8000, int retries = 1);

  TaskBuilder &gripper(const char *label, task_step_library::GripperCommand command);

  TaskBuilder &delay(const char *label, int delay_ms);

  task_step_library::TaskPlan build() &&;
  task_step_library::TaskPlan build() const &;

private:
  task_step_library::TaskPlan plan_{};
};

} // namespace task_orchestrator
