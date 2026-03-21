#pragma once

#include <string>

#include "shared_data/context.hpp"
#include "task_step_library/step.hpp"

namespace step_executor {

class TargetResolver {
public:
  bool resolve(const task_step_library::ArmMoveSpec &input,
               const task_step_library::SharedData &data,
               task_step_library::ArmMoveSpec &output,
               std::string &error) const;
};

} // namespace step_executor
