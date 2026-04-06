#pragma once

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "auto_library/method.hpp"
#include "step_executor/registry_bridge.hpp"

namespace engineer_auto::auto_node {

// ============================================================================
//  Capability Registry
// ----------------------------------------------------------------------------
//  - 将 auto kind 同时注册到 capability bridge 和 kind_specs
// ============================================================================

void registerAutoCapabilities(step_executor::RegistryBridge &bridge,
                              core::KindSpecMap &kind_specs,
                              rclcpp::Node &node);

} // namespace engineer_auto::auto_node
