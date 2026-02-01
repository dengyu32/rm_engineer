#pragma once

#include <map>
#include <sstream>
#include <string>

namespace error_code_utils {

enum class ErrorDomain {
  COMM = 0,
  SERVO = 1,
  SOLVE = 2,
  VISION = 3,
  UNKNOWN = 99,
};

struct Error {
  ErrorDomain domain{ErrorDomain::UNKNOWN};
  int code_value{0};
  std::string code_name;
  std::string message;
  std::map<std::string, std::string> context;
};

inline const char *domain_name(ErrorDomain domain) {
  switch (domain) {
  case ErrorDomain::COMM:
    return "COMM";
  case ErrorDomain::SERVO:
    return "SERVO";
  case ErrorDomain::SOLVE:
    return "SOLVE";
  case ErrorDomain::VISION:
    return "VISION";
  case ErrorDomain::UNKNOWN:
  default:
    return "UNKNOWN";
  }
}

inline std::string format_error(const Error &err) {
  std::ostringstream oss;
  oss << "domain=" << domain_name(err.domain);
  oss << " code_value=" << err.code_value;
  if (!err.code_name.empty()) {
    oss << " code_name=" << err.code_name;
  }
  if (!err.message.empty()) {
    oss << " message=\"" << err.message << "\"";
  }
  if (!err.context.empty()) {
    oss << " context={";
    bool first = true;
    for (const auto &kv : err.context) {
      if (!first) {
        oss << ", ";
      }
      first = false;
      oss << kv.first << ":" << kv.second;
    }
    oss << "}";
  }
  return oss.str();
}

} // namespace error_code_utils
