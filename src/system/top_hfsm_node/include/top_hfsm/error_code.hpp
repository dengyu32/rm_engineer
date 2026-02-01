#pragma once

#include <map>
#include <string>

#include "error_code_utils/error.hpp"
#include "error_code_utils/status_code_adapter.hpp"

namespace top_hfsm {

enum class ArmSolveClientErrc {
  ActionClientMissing = 0,
  ActionServerUnavailable,
  GoalRejected,
  EmptyResult,
};

inline SYSTEM_ERROR2_NAMESPACE::system_code to_system_code(ArmSolveClientErrc code) {
  using SYSTEM_ERROR2_NAMESPACE::errc;
  switch (code) {
  case ArmSolveClientErrc::ActionClientMissing:
    return errc::not_connected;
  case ArmSolveClientErrc::ActionServerUnavailable:
    return errc::host_unreachable;
  case ArmSolveClientErrc::GoalRejected:
    return errc::operation_not_permitted;
  case ArmSolveClientErrc::EmptyResult:
  default:
    return errc::state_not_recoverable;
  }
}

inline error_code_utils::Error make_error(
    ArmSolveClientErrc code,
    const std::string &message,
    const std::map<std::string, std::string> &context = {}) {
  auto sys = to_system_code(code);
  return error_code_utils::from_system_code(error_code_utils::ErrorDomain::SOLVE, sys,
                                            message, context);
}

} // namespace top_hfsm
