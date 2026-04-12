#pragma once

#include <array>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <engineer_interfaces/msg/slots.hpp>

#include "slot_select_node/slot_select_config.hpp"
#include "slot_select_node/slot_types.hpp"
#include "auto_library/command.hpp"
#include "auto_library/execute_result.hpp"

namespace engineer_auto::slot_select_node {

class SlotSelectNode {
public:
  explicit SlotSelectNode(rclcpp::Node &node, const SlotSelectConfig &config);

  core::ExecuteResult executeSelectPut();
  core::ExecuteResult executeSelectTake();
  core::ExecuteResult executeLock(const core::Command &cmd);
  core::ExecuteResult executeUnlock(const core::Command &cmd);
  bool selectSlot(SlotStrategy strategy, int &selected_slot);
  bool applySlotCommand(SlotStrategy strategy, int slot_id);
  std::string lastError() const { return last_error_; }

private:
  void onSlotState(const engineer_interfaces::msg::Slots::SharedPtr msg);
  void setSlotOccupied(int slot_id, bool occupied);
  std::array<bool, 2> slots() const;
  void publishSlotCommand(int slot_id, bool lock);
  int chooseFirstEmpty() const;
  int chooseFirstOccupied() const;
  bool isValidSlotId(int slot_id) const;

private:
  rclcpp::Node &node_;
  rclcpp::Logger logger_;
  SlotSelectConfig config_;
  std::string last_error_;

  mutable std::mutex mutex_;
  std::array<bool, 2> slots_{{false, false}};
  rclcpp::Subscription<engineer_interfaces::msg::Slots>::SharedPtr slot_state_sub_;
  rclcpp::Publisher<engineer_interfaces::msg::Slots>::SharedPtr slot_cmd_pub_;
};

} // namespace engineer_auto::slot_select_node
