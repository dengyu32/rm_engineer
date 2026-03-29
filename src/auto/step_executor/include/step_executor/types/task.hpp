#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "step_executor/types/step.hpp"

namespace step_executor {

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

} // namespace step_executor
