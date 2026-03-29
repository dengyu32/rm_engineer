#pragma once

#include <array>
#include <string>

#include "slot_select_node/slot_select_node.hpp"
#include "step_executor/types/capability_bridge.hpp"

namespace step_executor {

class SlotCapabilityBridge {
public:
  explicit SlotCapabilityBridge(rclcpp::Node &node);

  ExecuteResult run(const Command &cmd);
  void cancel();
  const char *lastError() const { return last_error_.c_str(); }

private:
  engineer_auto::slot_select_node::SlotSelectNode node_;
  std::string last_error_;
};

} // namespace step_executor
