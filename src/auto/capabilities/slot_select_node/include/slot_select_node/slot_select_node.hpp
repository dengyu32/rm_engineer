#pragma once

#include <array>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "task_step_library/step.hpp"

#include "engineer_interfaces/msg/slots.hpp"

namespace engineer_auto::slot_select_node {

struct SlotSelectConfig {
  std::string slot_state_topic{"/slot_states"};
  bool slot0_occupied{false};
  bool slot1_occupied{false};

  static SlotSelectConfig load(rclcpp::Node &node);
};

class SlotSelectNode {
public:
  explicit SlotSelectNode(rclcpp::Node &node, const SlotSelectConfig &config);

  bool selectSlot(task_step_library::SlotStrategy strategy, int &selected_slot);
  bool applySlotCommand(task_step_library::SlotStrategy strategy, int slot_id);
  void setSlotOccupied(int slot_id, bool occupied);
  std::array<bool, 2> slots() const;
  const std::string &lastError() const { return last_error_; }
  void cancel() {}

private:
  void onSlotState(const engineer_interfaces::msg::Slots::SharedPtr msg);
  void publishSlotCommand(int slot_id, bool lock);
  int chooseFirstEmpty() const;
  int chooseFirstOccupied() const;
  bool isValidSlotId(int slot_id) const;

private:
  rclcpp::Node &node_;
  rclcpp::Logger logger_;
  SlotSelectConfig config_;
  rclcpp::Subscription<engineer_interfaces::msg::Slots>::SharedPtr slot_state_sub_;
  rclcpp::Publisher<engineer_interfaces::msg::Slots>::SharedPtr slot_cmd_pub_;

  mutable std::mutex mutex_;
  std::array<bool, 2> slots_{{false, false}};
  std::string last_error_;
};

} // namespace engineer_auto::slot_select_node
