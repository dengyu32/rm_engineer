// Top HFSM
#include "top_hfsm/top_hfsm_node.hpp"
#include "top_hfsm/executors/gripper_control_node.hpp"
#include "top_hfsm/executors/moveit_servo_client.hpp"
#include "top_hfsm/top_hfsm_impl.hpp"

// ROS2
#include <rclcpp/node_options.hpp>
#include <vector>

namespace top_hfsm {

// ============================================================================
// ctor
// ============================================================================

TopHFSMNode::TopHFSMNode(const rclcpp::NodeOptions &options)
    : rclcpp::Node("top_hfsm_node", options),
      config_(TopHFSMConfig::Load(*this)),
      logger_(this->get_logger()),
      arm_solve_client_(*this, config_.arm_solve_client),
      gripper_control_node_(*this, config_.gripper),
      moveit_servo_client_(*this, config_.moveit_servo_client),
      machine_ctx_(&arm_solve_client_, &gripper_control_node_, &moveit_servo_client_, &dispatcher_, this->get_logger()),
      machine_{machine_ctx_} {
  error_bus_ = std::make_shared<error_code_utils::ErrorBus>();
  arm_solve_client_.set_error_bus(error_bus_);
  // --------------------------------------------------------------------------
  // Intent init
  // --------------------------------------------------------------------------
  latest_intent_.store(IntentType::IDLE, std::memory_order_relaxed);
  applied_intent_ = IntentType::IDLE;

  // --------------------------------------------------------------------------
  // Subscription: external intents
  // --------------------------------------------------------------------------
  intent_sub_ = this->create_subscription<engineer_interfaces::msg::Intent>(
      config_.intent_out_topic, rclcpp::QoS(10),
      std::bind(&TopHFSMNode::intentCallBack, this, std::placeholders::_1));

  // --------------------------------------------------------------------------
  // Timer: drive HFSM update loop
  // --------------------------------------------------------------------------
  machine_update_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(config_.hfsm_update_period_ms),
      std::bind(&TopHFSMNode::machine_update_timer_callback, this));

  dispatcher_.start();

  RCLCPP_INFO(this->get_logger(), "\n%s", config_.summary().c_str());
  RCLCPP_INFO(logger_, "[TOP_HFSM] TopHFSMNode started");
}

TopHFSMNode::~TopHFSMNode() { dispatcher_.stop(); }

// ============================================================================
//  ROS callbacks
// ============================================================================

void TopHFSMNode::intentCallBack(engineer_interfaces::msg::Intent::ConstSharedPtr msg) {

  // --------------------------------------------------------------------------
  // Defensive: validate intent_id
  // --------------------------------------------------------------------------
  const auto raw = static_cast<uint8_t>(msg->intent_id);

  if (raw > static_cast<uint8_t>(IntentType::TELEOP_SERVO)) {
    RCLCPP_WARN(this->get_logger(), "[scope=HFSM][status=invalid] invalid intent_id=%u, ignored", raw);
    return;
  }

  latest_intent_.store(static_cast<IntentType>(raw), std::memory_order_relaxed);
}

void TopHFSMNode::machine_update_timer_callback() {
  if (error_bus_) {
    std::vector<error_code_utils::Error> errors;
    error_bus_->drain(errors);
    for (const auto &err : errors) {
      RCLCPP_WARN(get_logger(), "[scope=HFSM][status=error] %s",
                  error_code_utils::format_error(err).c_str());
    }
  }

  std::vector<top_hfsm::AsyncDispatcher::Result> results;
  dispatcher_.poll(results);
  for (std::size_t i = 0; i < results.size(); ++i) {
    moveit_servo_client_.applyAsyncResult(results[i]);
  }

  const auto intent = latest_intent_.load(std::memory_order_relaxed);

  // --------------------------------------------------------------------------
  // 只在 intent 变化时 changeTo（避免每 tick 重复 changeTo）
  // --------------------------------------------------------------------------
  if (intent != applied_intent_) {
    applied_intent_ = intent;

    switch (intent) {
    case IntentType::AUTO_INIT:
      RCLCPP_INFO(get_logger(), "[scope=HFSM][status=intent] intent=AUTO_INIT");
      machine_.changeTo<AUTOINIT>();
      break;

    case IntentType::TEST_SOLVE:
      RCLCPP_INFO(get_logger(), "[scope=HFSM][status=intent] intent=TEST_SOLVE");
      machine_.changeTo<TESTSOLVE>();
      break;

    case IntentType::TEST_CARTESIAN:
      RCLCPP_INFO(get_logger(), "[HFSM] intent=TEST_CARTESIAN");
      machine_.changeTo<TESTCARTESIAN>();
      break;

    case IntentType::AUTO_GRAB:
      RCLCPP_INFO(get_logger(), "[HFSM] intent=AUTO_GRAB");
      machine_.changeTo<AUTOGRAB>();
      break;

    case IntentType::AUTO_STORE:
      RCLCPP_INFO(get_logger(), "[HFSM] intent=AUTO_STORE");
      machine_.changeTo<AUTOSTORE>();
      break;

    case IntentType::AUTO_GET:
      RCLCPP_INFO(get_logger(), "[HFSM] intent=AUTO_GET");
      machine_.changeTo<AUTOGET>();
      break;

    case IntentType::TELEOP_SERVO:
      RCLCPP_INFO(get_logger(), "[scope=HFSM][status=intent] intent=TELEOP_SERVO");
      machine_.changeTo<TELEOPSERVO>();
      break;

    case IntentType::IDLE:
      RCLCPP_WARN(get_logger(), "[scope=HFSM][status=intent] intent=IDLE");
      machine_.changeTo<IDLE>();
      break;

    default:
      // 已做范围保护，理论不会到这里；留空即可
      break;
    }
  }

  // --------------------------------------------------------------------------
  // 每 tick 推进一次 HFSM（内部状态执行）
  // --------------------------------------------------------------------------
  machine_.update();
}

} // namespace top_hfsm

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(top_hfsm::TopHFSMNode)
