#pragma once

#include <map>
#include <string>

#include "error_code_utils/error.hpp"
#include "error_code_utils/status_code_adapter.hpp"

namespace arm_servo {

enum class ArmServoErrc {
  ThrottleDrop = 0,
  TrajectoryEmpty,
  TrajectoryPointEmpty,
  IncompleteBeforeSeed,
};

inline SYSTEM_ERROR2_NAMESPACE::system_code to_system_code(ArmServoErrc code) {
  using SYSTEM_ERROR2_NAMESPACE::errc;
  switch (code) {
  case ArmServoErrc::ThrottleDrop:
    return errc::resource_unavailable_try_again;
  case ArmServoErrc::TrajectoryEmpty:
  case ArmServoErrc::TrajectoryPointEmpty:
  case ArmServoErrc::IncompleteBeforeSeed:
  default:
    return errc::invalid_argument;
  }
}

inline error_code_utils::Error make_error(
    ArmServoErrc code,
    const std::string &message,
    const std::map<std::string, std::string> &context = {}) {
  auto sys = to_system_code(code);
  return error_code_utils::from_system_code(error_code_utils::ErrorDomain::SERVO, sys,
                                            message, context);
}

} // namespace arm_servo
