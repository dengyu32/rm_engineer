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

  TaskBuilder &add(core::Step step);

  TaskBuilder &joints(const char *label, const std::array<float, 6> &target,
                      int timeout_ms = 8000, int retries = 1);

  TaskBuilder &vector(const char *label, const geometry_msgs::msg::Vector3 &target,
                      double target_length_m,
                      int timeout_ms = 8000, int retries = 1);

  TaskBuilder &slot(const char *label, task_step_library::SlotStrategy strategy,
                    int slot_id = -1);

  TaskBuilder &slot_mapped_joints(const char *label, int timeout_ms = 8000, int retries = 1);

  TaskBuilder &vision(const char *label, int timeout_ms = 3000, int retries = 0);

  TaskBuilder &vision_mapped_pose(const char *label, int timeout_ms = 8000, int retries = 1);

  TaskBuilder &vision_mapped_vector(const char *label, double target_length_m,
                                    int timeout_ms = 8000, int retries = 1);

  TaskBuilder &gripper(const char *label, task_step_library::GripperCommand command);

  TaskBuilder &delay(const char *label, int delay_ms);

  task_step_library::TaskPlan build() &&;
  task_step_library::TaskPlan build() const &;

private:
  core::TaskPlan plan_{};
};

} // namespace task_orchestrator
