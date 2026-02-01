#pragma once

#include <map>
#include <string>

#include "error_code_utils/error.hpp"
#include "error_code_utils/status_code_adapter.hpp"

namespace solve_core {

enum class SolveErrc {
  AdapterMissing = 0,
  RobotModelMissing,
  JointModelGroupMissing,
  IkSolverMissing,
  StartStateInvalid,
  InvalidRequest,
  IkFailed,
  PlanFailed,
  TargetSizeMismatch,
  JointStateMissing,
  TimeParameterizationFailed,
  CollisionDetected,
  JointLookupFailed,
  UnknownOption,
};

inline bool is_retryable(SolveErrc code) {
  switch (code) {
  case SolveErrc::PlanFailed:
  case SolveErrc::TimeParameterizationFailed:
    return true;
  default:
    return false;
  }
}

inline bool is_fatal(SolveErrc code) {
  switch (code) {
  case SolveErrc::AdapterMissing:
  case SolveErrc::RobotModelMissing:
    return true;
  default:
    return false;
  }
}

inline SYSTEM_ERROR2_NAMESPACE::system_code to_system_code(SolveErrc code) {
  using SYSTEM_ERROR2_NAMESPACE::errc;
  switch (code) {
  case SolveErrc::AdapterMissing:
    return errc::not_connected;
  case SolveErrc::RobotModelMissing:
    return errc::not_connected;
  case SolveErrc::JointModelGroupMissing:
    return errc::invalid_argument;
  case SolveErrc::IkSolverMissing:
    return errc::not_supported;
  case SolveErrc::StartStateInvalid:
    return errc::invalid_argument;
  case SolveErrc::InvalidRequest:
    return errc::invalid_argument;
  case SolveErrc::IkFailed:
    return errc::invalid_argument;
  case SolveErrc::PlanFailed:
    return errc::state_not_recoverable;
  case SolveErrc::TargetSizeMismatch:
    return errc::invalid_argument;
  case SolveErrc::JointStateMissing:
    return errc::invalid_argument;
  case SolveErrc::TimeParameterizationFailed:
    return errc::state_not_recoverable;
  case SolveErrc::CollisionDetected:
    return errc::state_not_recoverable;
  case SolveErrc::JointLookupFailed:
    return errc::invalid_argument;
  case SolveErrc::UnknownOption:
  default:
    return errc::invalid_argument;
  }
}

inline error_code_utils::Error make_error(
    SolveErrc code,
    const std::string &message,
    const std::map<std::string, std::string> &context = {}) {
  auto sys = to_system_code(code);
  return error_code_utils::from_system_code(error_code_utils::ErrorDomain::SOLVE, sys,
                                            message, context);
}

} // namespace solve_core
