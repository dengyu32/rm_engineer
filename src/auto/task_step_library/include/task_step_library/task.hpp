#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "task_step_library/step.hpp"

namespace task_step_library {

// TASK 

enum class TaskId : uint8_t {
  IDLE = 0,
  AUTO_INIT = 1,
  AUTO_GRAB = 2,
  AUTO_STORE = 3,
  AUTO_GET = 4,
  TEST_SOLVE = 5,
  TEST_CARTESIAN = 6,
};

enum class TaskFinishCode : uint8_t {
  Running = 0,
  Finished = 1,
  Aborted = 2,
};

struct TaskRequest {
  TaskId task_id{TaskId::IDLE};
};

struct TaskPlan {
  TaskId task_id{TaskId::IDLE};
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

inline bool is_supported_task(TaskId id) {
  switch (id) {
  case TaskId::IDLE:
  case TaskId::AUTO_INIT:
  case TaskId::AUTO_GRAB:
  case TaskId::AUTO_STORE:
  case TaskId::AUTO_GET:
  case TaskId::TEST_SOLVE:
  case TaskId::TEST_CARTESIAN:
    return true;
  default:
    return false;
  }
}

} // namespace task_step_library
