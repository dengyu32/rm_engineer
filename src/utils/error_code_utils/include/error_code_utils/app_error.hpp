#pragma once

#include <map>
#include <string>

#include "error_code_utils/error.hpp"

namespace error_code_utils {
namespace app {

// COMM domain codes (USB, fake system, etc.)
enum class CommCode : int {
  UsbOpenFailed = 100,
  UsbOpenException = 101,
  UsbSendFailed = 102,
  UsbPacketTooSmall = 103,
  FakeJointCommandShort = 200,
};

// SERVO domain codes
enum class ServoCode : int {
  ThrottleDrop = 100,
  TrajectoryEmpty = 101,
  TrajectoryPointEmpty = 102,
  IncompleteBeforeSeed = 103,
};

// SOLVE domain codes (solve_core + arm_solve + hfsm client)
enum class SolveCode : int {
  // solve_core: 100-199
  AdapterMissing = 100,
  RobotModelMissing = 101,
  JointModelGroupMissing = 102,
  IkSolverMissing = 103,
  StartStateInvalid = 104,
  InvalidRequest = 105,
  IkFailed = 106,
  PlanFailed = 107,
  TargetSizeMismatch = 108,
  JointStateMissing = 109,
  TimeParameterizationFailed = 110,
  CollisionDetected = 111,
  JointLookupFailed = 112,
  UnknownOption = 113,

  // arm_solve_server: 200-299
  MoveGroupNotReady = 200,
  SolveCoreNotReady = 201,
  PlanningThrottled = 202,
  PlanningFailed = 203,
  GoalCanceled = 204,
  TrajectoryEmpty = 205,
  TrajectoryPointMismatch = 206,
  ExceptionThrown = 207,

  // top_hfsm arm_solve_client: 300-399
  ActionClientMissing = 300,
  ActionServerUnavailable = 301,
  GoalRejected = 302,
  EmptyResult = 303,
};

inline const char *to_string(CommCode code) {
  switch (code) {
  case CommCode::UsbOpenFailed:
    return "UsbOpenFailed";
  case CommCode::UsbOpenException:
    return "UsbOpenException";
  case CommCode::UsbSendFailed:
    return "UsbSendFailed";
  case CommCode::UsbPacketTooSmall:
    return "UsbPacketTooSmall";
  case CommCode::FakeJointCommandShort:
    return "FakeJointCommandShort";
  default:
    return "CommUnknown";
  }
}

inline const char *to_string(ServoCode code) {
  switch (code) {
  case ServoCode::ThrottleDrop:
    return "ThrottleDrop";
  case ServoCode::TrajectoryEmpty:
    return "TrajectoryEmpty";
  case ServoCode::TrajectoryPointEmpty:
    return "TrajectoryPointEmpty";
  case ServoCode::IncompleteBeforeSeed:
    return "IncompleteBeforeSeed";
  default:
    return "ServoUnknown";
  }
}

inline const char *to_string(SolveCode code) {
  switch (code) {
  case SolveCode::AdapterMissing:
    return "AdapterMissing";
  case SolveCode::RobotModelMissing:
    return "RobotModelMissing";
  case SolveCode::JointModelGroupMissing:
    return "JointModelGroupMissing";
  case SolveCode::IkSolverMissing:
    return "IkSolverMissing";
  case SolveCode::StartStateInvalid:
    return "StartStateInvalid";
  case SolveCode::InvalidRequest:
    return "InvalidRequest";
  case SolveCode::IkFailed:
    return "IkFailed";
  case SolveCode::PlanFailed:
    return "PlanFailed";
  case SolveCode::TargetSizeMismatch:
    return "TargetSizeMismatch";
  case SolveCode::JointStateMissing:
    return "JointStateMissing";
  case SolveCode::TimeParameterizationFailed:
    return "TimeParameterizationFailed";
  case SolveCode::CollisionDetected:
    return "CollisionDetected";
  case SolveCode::JointLookupFailed:
    return "JointLookupFailed";
  case SolveCode::UnknownOption:
    return "UnknownOption";
  case SolveCode::MoveGroupNotReady:
    return "MoveGroupNotReady";
  case SolveCode::SolveCoreNotReady:
    return "SolveCoreNotReady";
  case SolveCode::PlanningThrottled:
    return "PlanningThrottled";
  case SolveCode::PlanningFailed:
    return "PlanningFailed";
  case SolveCode::GoalCanceled:
    return "GoalCanceled";
  case SolveCode::TrajectoryEmpty:
    return "TrajectoryEmpty";
  case SolveCode::TrajectoryPointMismatch:
    return "TrajectoryPointMismatch";
  case SolveCode::ExceptionThrown:
    return "ExceptionThrown";
  case SolveCode::ActionClientMissing:
    return "ActionClientMissing";
  case SolveCode::ActionServerUnavailable:
    return "ActionServerUnavailable";
  case SolveCode::GoalRejected:
    return "GoalRejected";
  case SolveCode::EmptyResult:
    return "EmptyResult";
  default:
    return "SolveUnknown";
  }
}

inline Error make_app_error(ErrorDomain domain,
                            int code_value,
                            const char *code_name,
                            const std::string &message,
                            const std::map<std::string, std::string> &context = {}) {
  Error err;
  err.domain = domain;
  err.code_value = code_value;
  err.code_name = code_name ? code_name : "";
  err.message = message;
  err.context = context;
  return err;
}

template <typename Enum>
inline Error make_app_error(ErrorDomain domain,
                            Enum code,
                            const std::string &message,
                            const std::map<std::string, std::string> &context = {}) {
  return make_app_error(domain, static_cast<int>(code), to_string(code), message, context);
}

} // namespace app
} // namespace error_code_utils
