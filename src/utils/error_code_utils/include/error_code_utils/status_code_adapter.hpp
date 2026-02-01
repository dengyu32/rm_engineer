#pragma once

#include <map>
#include <string>

#include "error_code_utils/error.hpp"
#include "error_code_utils/system_error2.hpp"

namespace error_code_utils {

inline Error from_system_code(ErrorDomain domain,
                              const SYSTEM_ERROR2_NAMESPACE::system_code &code,
                              const std::string &message,
                              const std::map<std::string, std::string> &context = {}) {
  Error err;
  err.domain = domain;
  err.code_value = code.value();
  const auto msg_ref = code.message();
  err.code_name = std::string(msg_ref.data(), msg_ref.size());
  err.message = message;
  err.context = context;
  return err;
}

} // namespace error_code_utils
