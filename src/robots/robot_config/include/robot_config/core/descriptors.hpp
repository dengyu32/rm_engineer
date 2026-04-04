#pragma once

#include <rcl_interfaces/msg/parameter_descriptor.hpp>

#include <string>

namespace params_utils {
namespace detail {

inline rcl_interfaces::msg::ParameterDescriptor
make_descriptor(const std::string &description, bool read_only = false) {
  rcl_interfaces::msg::ParameterDescriptor desc;
  desc.description = description;
  desc.read_only = read_only;
  return desc;
}

}  // namespace detail
}  // namespace params_utils
