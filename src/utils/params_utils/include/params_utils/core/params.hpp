#pragma once

#include <rclcpp/rclcpp.hpp>

#include <stdexcept>
#include <string>

namespace params_utils {
namespace detail {

//--------------------------------------
// Basic helpers
//--------------------------------------
template <class NodeT, class T>
inline void declare_get(NodeT &node, const std::string &name, T &value) {
  if (!node.has_parameter(name)) {
    node.declare_parameter(name, value);
  }
  node.get_parameter(name, value);
}

template <class NodeT, class T, class Check>
inline void declare_get_checked(NodeT &node,
                                const std::string &name,
                                T &value,
                                Check check,
                                const char *msg) {
  declare_get(node, name, value);
  if (!check(value)) {
    throw std::runtime_error(std::string("Config: ") + name + ": " + msg);
  }
}

}  // namespace detail
}  // namespace params_utils
