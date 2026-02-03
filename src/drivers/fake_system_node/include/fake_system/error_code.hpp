#pragma once

#include <map>
#include <string>

#include "error_code_utils/error.hpp"
#include "error_code_utils/status_code_adapter.hpp"

namespace fake_system {

enum class FakeSystemErrc {
  JointCommandShort = 0,  // 
};

inline SYSTEM_ERROR2_NAMESPACE::system_code to_system_code(FakeSystemErrc code) {
  using SYSTEM_ERROR2_NAMESPACE::errc;
  switch (code) {
  case FakeSystemErrc::JointCommandShort:
  default:
    return errc::invalid_argument;
  }
}

inline error_code_utils::Error make_error(
    FakeSystemErrc code,
    const std::string &message,
    const std::map<std::string, std::string> &context = {}) {
  auto sys = to_system_code(code);
  return error_code_utils::from_system_code(error_code_utils::ErrorDomain::COMM, sys,
                                            message, context);
}

// 示例： return make_error(FakeSystemErrc::JointCommandShort, "joint command length mismatch", {{"expected", "6"}, {"actual", "4"}});

} // namespace fake_system
