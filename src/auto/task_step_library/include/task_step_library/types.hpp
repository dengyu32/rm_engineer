#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include <engineer_interfaces/msg/target.hpp>

namespace task_step_library {

enum class TaskId : uint8_t {
  IDLE = 0,
  AUTO_INIT = 1,
  TEST_SOLVE = 2,
  TEST_CARTESIAN = 3,
  AUTO_GRAB = 4,
  AUTO_STORE = 5,
  AUTO_GET = 6,
};

enum class TaskFinishCode : uint8_t {
  Running = 0,
  Finished = 1,
  Aborted = 2,
};

struct TaskRequest {
  TaskId task_id{TaskId::IDLE};
};

enum class PlanOption : uint8_t {
  NORMAL = 0,
  CARTESIAN = 1,
  JOINTS = 2,
};

enum class StepType : uint8_t {
  ArmMove = 0,
  Gripper = 1,
  Delay = 2,
  Guard = 3,
};

enum class GripperCommand : uint8_t {
  OPEN = 0,
  CLOSE = 1,
};

enum class TaskStatus : uint8_t {
  Success = 0,
  Failure = 1,
  Timeout = 2,
  Canceled = 3,
  Running = 4,
};

struct ArmMoveSpec {
  engineer_interfaces::msg::Target pose{};
  std::array<float, 6> joints{{0.f, 0.f, 0.f, 0.f, 0.f, 0.f}};
  PlanOption plan_option{PlanOption::NORMAL};
};

struct GripperSpec {
  GripperCommand command{GripperCommand::OPEN};
};

struct Step {
  StepType type{StepType::Delay};
  std::string label{};
  int timeout_ms{0};
  int max_retries{0};

  ArmMoveSpec arm_move{};
  GripperSpec gripper{};
  int delay_ms{0};
  int guard_id{0};
};

struct TaskPlan {
  TaskId task_id{TaskId::IDLE};
  std::vector<Step> steps;
};

struct TaskResult {
  TaskStatus status{TaskStatus::Running};
  std::string message;
};

inline bool is_supported_task(TaskId id) {
  switch (id) {
  case TaskId::IDLE:
  case TaskId::AUTO_INIT:
  case TaskId::TEST_SOLVE:
  case TaskId::TEST_CARTESIAN:
  case TaskId::AUTO_GRAB:
  case TaskId::AUTO_STORE:
  case TaskId::AUTO_GET:
    return true;
  default:
    return false;
  }
}

} // namespace task_step_library
