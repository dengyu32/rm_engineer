#pragma once

#include <rclcpp/rclcpp.hpp>

#include <sstream>
#include <string>
#include <vector>

namespace param_utils {

struct ParamKV {
  std::string key;
  rclcpp::Parameter value;
};

inline std::vector<ParamKV> CollectAllParams(rclcpp::Node& node, int depth = 100) {
  std::vector<ParamKV> out;

  const auto result = node.list_parameters({}, depth);
  out.reserve(result.names.size());
  for (const auto& name : result.names) {
    rclcpp::Parameter p;
    if (node.get_parameter(name, p)) {
      out.push_back({name, p});
    }
  }
  return out;
}

inline std::string ToString(const rclcpp::Parameter& p) {
  using T = rclcpp::ParameterType;

  switch (p.get_type()) {
    case T::PARAMETER_NOT_SET:
      return "<not_set>";

    case T::PARAMETER_BOOL:
      return p.as_bool() ? "true" : "false";

    case T::PARAMETER_INTEGER:
      return std::to_string(p.as_int());

    case T::PARAMETER_DOUBLE:
      return std::to_string(p.as_double());

    case T::PARAMETER_STRING:
      return p.as_string();

    case T::PARAMETER_BOOL_ARRAY: {
      const auto v = p.as_bool_array();
      std::ostringstream oss;
      oss << "[";
      for (std::size_t i = 0; i < v.size(); ++i) {
        oss << (v[i] ? "true" : "false");
        if (i + 1 < v.size()) {
          oss << ", ";
        }
      }
      oss << "]";
      return oss.str();
    }

    case T::PARAMETER_INTEGER_ARRAY: {
      const auto v = p.as_integer_array();
      std::ostringstream oss;
      oss << "[";
      for (std::size_t i = 0; i < v.size(); ++i) {
        oss << v[i];
        if (i + 1 < v.size()) {
          oss << ", ";
        }
      }
      oss << "]";
      return oss.str();
    }

    case T::PARAMETER_DOUBLE_ARRAY: {
      const auto v = p.as_double_array();
      std::ostringstream oss;
      oss << "[";
      for (std::size_t i = 0; i < v.size(); ++i) {
        oss << v[i];
        if (i + 1 < v.size()) {
          oss << ", ";
        }
      }
      oss << "]";
      return oss.str();
    }

    case T::PARAMETER_STRING_ARRAY: {
      const auto v = p.as_string_array();
      std::ostringstream oss;
      oss << "[";
      for (std::size_t i = 0; i < v.size(); ++i) {
        oss << v[i];
        if (i + 1 < v.size()) {
          oss << ", ";
        }
      }
      oss << "]";
      return oss.str();
    }

    default:
      return "<unsupported_type>";
  }
}

inline void LogSnapshot(const rclcpp::Logger& logger,
                        const std::vector<ParamKV>& snapshot,
                        const std::string& prefix = "[param] ") {
  RCLCPP_INFO(logger, " ");
  RCLCPP_INFO(logger, "-------------------------------------------------");
  for (const auto& kv : snapshot) {
    RCLCPP_INFO(logger, "%s%s: %s", prefix.c_str(), kv.key.c_str(),
                ToString(kv.value).c_str());
  }
  RCLCPP_INFO(logger, "-------------------------------------------------");
  RCLCPP_INFO(logger, " ");
}

inline std::string DumpSnapshotText(const std::vector<ParamKV>& snapshot) {
  std::ostringstream oss;
  for (const auto& kv : snapshot) {
    oss << kv.key << ": " << ToString(kv.value) << '\n';
  }
  return oss.str();
}

}  // namespace param_utils
