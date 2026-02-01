#pragma once

#include <map>
#include <string>

#include "error_code_utils/error.hpp"
#include "error_code_utils/status_code_adapter.hpp"

namespace arm_solve {

enum class ArmSolveErrc {
  MoveGroupNotReady = 0,
  SolveCoreNotReady,
  PlanningThrottled,
  PlanningFailed,
  GoalCanceled,
  TrajectoryEmpty,
  TrajectoryPointMismatch,
  ExceptionThrown,
};

inline SYSTEM_ERROR2_NAMESPACE::system_code to_system_code(ArmSolveErrc code) {
  using SYSTEM_ERROR2_NAMESPACE::errc;
  switch (code) {
  case ArmSolveErrc::MoveGroupNotReady:
    return errc::not_connected;
  case ArmSolveErrc::SolveCoreNotReady:
    return errc::resource_unavailable_try_again;
  case ArmSolveErrc::PlanningThrottled:
    return errc::resource_unavailable_try_again;
  case ArmSolveErrc::PlanningFailed:
    return errc::state_not_recoverable;
  case ArmSolveErrc::GoalCanceled:
    return errc::operation_canceled;
  case ArmSolveErrc::TrajectoryEmpty:
  case ArmSolveErrc::TrajectoryPointMismatch:
  case ArmSolveErrc::ExceptionThrown:
  default:
    return errc::invalid_argument;
  }
}

inline error_code_utils::Error make_error(
    ArmSolveErrc code,
    const std::string &message,
    const std::map<std::string, std::string> &context = {}) {
  auto sys = to_system_code(code);
  return error_code_utils::from_system_code(error_code_utils::ErrorDomain::SOLVE, sys,
                                            message, context);
}

} // namespace arm_solve
