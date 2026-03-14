#include "slot_select_node/slot_select_node.hpp"

#include <cstddef>

namespace engineer_auto::slot_select_node {

SlotSelectNode::SlotSelectNode(rclcpp::Node &node, const SlotSelectConfig &config)
    : node_(node), logger_(node.get_logger()), config_(config) {
  RCLCPP_INFO(logger_, "\n%s", config_.summary().c_str());
  {
    std::scoped_lock lock(mutex_);
    slots_[0] = config_.slot0_occupied;
    slots_[1] = config_.slot1_occupied;
  }

  slot_state_sub_ = node_.create_subscription<engineer_interfaces::msg::Slots>(
      config_.slot_state_topic, rclcpp::QoS(10),
      std::bind(&SlotSelectNode::onSlotState, this, std::placeholders::_1));
  slot_cmd_pub_ = node_.create_publisher<engineer_interfaces::msg::Slots>(
      "/slot_cmds", rclcpp::QoS(10));

  RCLCPP_INFO(logger_, "[SLOT_SELECT] started topic=%s init=[%d,%d]",
              config_.slot_state_topic.c_str(), slots_[0] ? 1 : 0, slots_[1] ? 1 : 0);
}

bool SlotSelectNode::selectSlot(task_step_library::SlotStrategy strategy, int &selected_slot) {
  std::scoped_lock lock(mutex_);

  selected_slot = -1;
  switch (strategy) {
  case task_step_library::SlotStrategy::SelectSlotToPut:
    selected_slot = chooseFirstEmpty();
    if (selected_slot < 0) {
      last_error_ = "no empty slot";
      return false;
    }
    break;
  case task_step_library::SlotStrategy::SelectSlotToTake:
    selected_slot = chooseFirstOccupied();
    if (selected_slot < 0) {
      last_error_ = "no occupied slot";
      return false;
    }
    break;
  default:
    last_error_ = "unsupported select slot strategy";
    return false;
  }

  last_error_.clear();
  return true;
}

bool SlotSelectNode::applySlotCommand(task_step_library::SlotStrategy strategy, int slot_id) {
  std::scoped_lock lock(mutex_);

  if (!isValidSlotId(slot_id)) {
    last_error_ = "lock/unlock requires valid slot id";
    return false;
  }

  switch (strategy) {
  case task_step_library::SlotStrategy::LockSlot:
    publishSlotCommand(slot_id, true);
    break;
  case task_step_library::SlotStrategy::UnlockSlot:
    publishSlotCommand(slot_id, false);
    break;
  default:
    last_error_ = "unsupported slot command strategy";
    return false;
  }

  last_error_.clear();
  return true;
}

void SlotSelectNode::setSlotOccupied(int slot_id, bool occupied) {
  std::scoped_lock lock(mutex_);
  if (!isValidSlotId(slot_id)) {
    last_error_ = "invalid slot id";
    return;
  }
  slots_[static_cast<std::size_t>(slot_id)] = occupied;
  last_error_.clear();
}

std::array<bool, 2> SlotSelectNode::slots() const {
  std::scoped_lock lock(mutex_);
  return slots_;
}

void SlotSelectNode::publishSlotCommand(int slot_id, bool lock) {
  if (!slot_cmd_pub_) {
    return;
  }

  engineer_interfaces::msg::Slots cmd;
  cmd.header.stamp = node_.now();
  cmd.slots.resize(slots_.size());
  for (std::size_t i = 0; i < slots_.size(); ++i) {
    cmd.slots[i].status = slots_[i];
    cmd.slots[i].command = false;
  }

  if (isValidSlotId(slot_id)) {
    cmd.slots[static_cast<std::size_t>(slot_id)].command = lock;
  }
  slot_cmd_pub_->publish(cmd);
}

void SlotSelectNode::onSlotState(const engineer_interfaces::msg::Slots::SharedPtr msg) {
  if (!msg || msg->slots.size() < slots_.size()) {
    return;
  }

  std::scoped_lock lock(mutex_);
  slots_[0] = msg->slots[0].status;
  slots_[1] = msg->slots[1].status;
}

int SlotSelectNode::chooseFirstEmpty() const {
  for (std::size_t i = 0; i < slots_.size(); ++i) {
    if (!slots_[i]) {
      return static_cast<int>(i);
    }
  }
  return -1;
}

int SlotSelectNode::chooseFirstOccupied() const {
  for (std::size_t i = 0; i < slots_.size(); ++i) {
    if (slots_[i]) {
      return static_cast<int>(i);
    }
  }
  return -1;
}

bool SlotSelectNode::isValidSlotId(int slot_id) const {
  return slot_id >= 0 && slot_id < static_cast<int>(slots_.size());
}

} // namespace engineer_auto::slot_select_node
