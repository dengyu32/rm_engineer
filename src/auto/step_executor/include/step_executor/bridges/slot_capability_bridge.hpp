#pragma once

#include <string>

#include "slot_select_node/slot_select_node.hpp"
#include "step_executor/capability_bridge.hpp"

namespace step_executor {

class SlotCapabilityBridge {
public:
  explicit SlotCapabilityBridge(rclcpp::Node &node);

  BridgeResult runSlotStep(const task_step_library::Step &step,
                           task_step_library::StepResult *out_result);
  void cancel() {}
  const char *lastError() const { return last_error_.c_str(); }
  int lastSelectedSlot() const { return last_selected_slot_; }

private:
  engineer_auto::slot_select_node::SlotSelectNode node_;
  int last_selected_slot_{-1};
  std::string last_error_;
};

} // namespace step_executor
