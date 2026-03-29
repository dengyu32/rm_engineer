#pragma once

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "step_executor/types/capability_bridge.hpp"

namespace engineer_auto::auto_bridge {

// ============================================================================
//  Capability Registry
// ----------------------------------------------------------------------------
//  - 将 kind 映射到具体 capability bridge
// ============================================================================

std::shared_ptr<step_executor::ICapabilityBridge> createAutoCapabilityBridge(
    rclcpp::Node &node);

} // namespace engineer_auto::auto_bridge
