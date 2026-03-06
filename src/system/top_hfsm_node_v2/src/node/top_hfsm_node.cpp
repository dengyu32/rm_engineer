#include "top_hfsm_v2/node/top_hfsm_node.hpp"

#include <chrono>
#include <vector>

#include <rclcpp/node_options.hpp>

#include "top_hfsm_v2/plans/auto_get_plan.hpp"
#include "top_hfsm_v2/plans/auto_grab_plan.hpp"
#include "top_hfsm_v2/plans/auto_init_plan.hpp"
#include "top_hfsm_v2/plans/auto_store_plan.hpp"
#include "top_hfsm_v2/plans/test_cartesian_plan.hpp"
#include "top_hfsm_v2/plans/test_solve_plan.hpp"

namespace top_hfsm_v2 {

NodeConfig NodeConfig::Load(rclcpp::Node &node) {
  NodeConfig config;
  auto declare_get = [&](const std::string &name, auto &value) {
    node.declare_parameter(name, value);
    node.get_parameter(name, value);
  };

  declare_get("intent_cmd_topic", config.intent_cmd_topic);
  declare_get("intent_fb_topic", config.intent_fb_topic);
  declare_get("hfsm_update_period_ms", config.hfsm_update_period_ms);

  config.arm_solve_client = ArmSolveClientConfig::Load(node);
  config.moveit_servo_client = MoveItServoClientConfig::Load(node);
  config.gripper = GripperConfig::Load(node);

  config.validate();
  return config;
}

void NodeConfig::validate() const {
  if (intent_cmd_topic.empty() || intent_fb_topic.empty()) {
    throw std::runtime_error("NodeConfig: intent topics must not be empty");
  }
  if (hfsm_update_period_ms < 1 || hfsm_update_period_ms > 1000) {
    throw std::runtime_error(
        "NodeConfig: hfsm_update_period_ms must be in [1,1000]");
  }
}

TopHFSMNode::TopHFSMNode(const rclcpp::NodeOptions &options)
    : rclcpp::Node("top_hfsm_v2_node", options),
      config_(NodeConfig::Load(*this)),
      logger_(this->get_logger()),
      arm_solve_client_(*this, config_.arm_solve_client),
      gripper_control_node_(*this, config_.gripper),
      moveit_servo_client_(*this, config_.moveit_servo_client) {
  error_bus_ = std::make_shared<error_code_utils::ErrorBus>();
  arm_solve_client_.set_error_bus(error_bus_);

  runtime_context_.arm_client = &arm_solve_client_;
  runtime_context_.gripper_node = &gripper_control_node_;
  runtime_context_.servo_client = &moveit_servo_client_;
  runtime_context_.logger = logger_;

  intent_sub_ = this->create_subscription<engineer_interfaces::msg::Intent>(
      config_.intent_cmd_topic, rclcpp::QoS(10),
      std::bind(&TopHFSMNode::intentCallback, this, std::placeholders::_1));
  intent_fb_pub_ = this->create_publisher<engineer_interfaces::msg::Intent>(
      config_.intent_fb_topic, rclcpp::QoS(10));
  update_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(config_.hfsm_update_period_ms),
      std::bind(&TopHFSMNode::tick, this));

  enterIdle();
  RCLCPP_INFO(logger_, "[TOP_HFSM_V2] started");
}

TopHFSMNode::~TopHFSMNode() = default;

void TopHFSMNode::intentCallback(
    engineer_interfaces::msg::Intent::ConstSharedPtr msg) {
  const uint8_t raw = static_cast<uint8_t>(msg->intent_id);
  if (!isSupportedIntent(raw)) {
    RCLCPP_WARN(logger_, "[TOP_HFSM_V2] invalid intent_id=%u ignored", raw);
    return;
  }
  latest_intent_.store(static_cast<IntentType>(raw), std::memory_order_relaxed);
}

void TopHFSMNode::tick() {
  if (error_bus_) {
    std::vector<error_code_utils::Error> errors;
    error_bus_->drain(errors);
    for (const auto &err : errors) {
      RCLCPP_WARN(logger_, "[TOP_HFSM_V2][error] %s",
                  error_code_utils::format_error(err).c_str());
    }
  }

  const IntentType next_intent = latest_intent_.load(std::memory_order_relaxed);
  if (next_intent != applied_intent_) {
    handleIntentTransition(next_intent);
    applied_intent_ = next_intent;
  }

  if (active_mode_ == ActiveMode::CommandSequence) {
    if (!command_sequence_.isFinished()) {
      static rclcpp::Clock steady_clock(RCL_STEADY_TIME);
      command_sequence_.update(runtime_context_, steady_clock.now());
    }

    if (command_sequence_.isFinished() &&
        last_finish_code_ == FinishCode::Running) {
      const bool success = command_sequence_.result() == TaskStatus::Success;
      last_finish_code_ = success ? FinishCode::Finished : FinishCode::Aborted;
      publishIntentFeedback(static_cast<uint8_t>(active_task_intent_),
                            last_finish_code_);
      const char *msg = command_sequence_.reason().empty()
                            ? (success ? "sequence finished"
                                       : "sequence aborted")
                            : command_sequence_.reason().c_str();
      RCLCPP_INFO(logger_, "[TOP_HFSM_V2] intent=%u done code=%u msg=%s",
                  static_cast<unsigned>(active_task_intent_),
                  static_cast<unsigned>(last_finish_code_), msg);
    }
    return;
  }

  if (active_mode_ == ActiveMode::TeleopServo &&
      last_finish_code_ != FinishCode::Running) {
    last_finish_code_ = FinishCode::Running;
    publishIntentFeedback(static_cast<uint8_t>(active_task_intent_),
                          FinishCode::Running);
  }
}

void TopHFSMNode::publishIntentFeedback(uint8_t intent_id,
                                        FinishCode finish_code) {
  engineer_interfaces::msg::Intent msg;
  msg.stamp = this->now();
  msg.intent_id = intent_id;
  msg.intent_finish = static_cast<uint8_t>(finish_code);
  intent_fb_pub_->publish(msg);
}

void TopHFSMNode::handleIntentTransition(IntentType next_intent) {
  if (active_mode_ == ActiveMode::CommandSequence &&
      !command_sequence_.isFinished() && next_intent != active_task_intent_) {
    abortCurrentIntent(FinishCode::Aborted, "preempted by new intent");
  }

  if (active_mode_ == ActiveMode::TeleopServo &&
      next_intent != IntentType::TELEOP_SERVO) {
    // Realtime servo chain is intentionally disabled for teleop collision-guard mode.
    // Keep old logic for quick rollback if needed:
    // if (runtime_context_.servo_client) {
    //   (void)runtime_context_.servo_client->pauseServo();
    // }
  }

  switch (next_intent) {
  case IntentType::IDLE:
    enterIdle();
    return;
  case IntentType::TELEOP_SERVO:
    active_mode_ = ActiveMode::TeleopServo;
    active_task_intent_ = next_intent;
    last_finish_code_ = FinishCode::Running;
    // Realtime servo chain is intentionally disabled for teleop collision-guard mode.
    // Keep old logic for quick rollback if needed:
    // if (runtime_context_.servo_client) {
    //   (void)runtime_context_.servo_client->startServo();
    // }
    publishIntentFeedback(static_cast<uint8_t>(next_intent),
                          FinishCode::Running);
    return;
  default:
    beginSequenceForIntent(next_intent);
    return;
  }
}

void TopHFSMNode::beginSequenceForIntent(IntentType intent) {
  std::vector<CommandStep> steps;

  switch (intent) {
  case IntentType::AUTO_INIT:
    steps = makeAutoInitPlan();
    break;
  case IntentType::TEST_SOLVE:
    steps = makeTestSolvePlan();
    break;
  case IntentType::TEST_CARTESIAN:
    steps = makeTestCartesianPlan();
    break;
  case IntentType::AUTO_GRAB:
    steps = makeAutoGrabPlan();
    break;
  case IntentType::AUTO_STORE:
    steps = makeAutoStorePlan();
    break;
  case IntentType::AUTO_GET:
    steps = makeAutoGetPlan();
    break;
  default:
    enterIdle();
    return;
  }

  command_sequence_.setSteps(steps);
  command_sequence_.reset();
  active_task_intent_ = intent;
  active_mode_ = ActiveMode::CommandSequence;
  last_finish_code_ = FinishCode::Running;
  publishIntentFeedback(static_cast<uint8_t>(intent), FinishCode::Running);
}

void TopHFSMNode::enterIdle() {
  command_sequence_.reset();
  active_task_intent_ = IntentType::IDLE;
  active_mode_ = ActiveMode::Idle;
  last_finish_code_ = FinishCode::Running;
  publishIntentFeedback(0, FinishCode::Running);
}

void TopHFSMNode::abortCurrentIntent(FinishCode finish_code,
                                     const char *reason) {
  if (active_task_intent_ == IntentType::IDLE) {
    return;
  }

  if (runtime_context_.arm_client) {
    runtime_context_.arm_client->sendCancel();
  }
  if (runtime_context_.gripper_node) {
    runtime_context_.gripper_node->setGripperPosition(GripperCommandType::OPEN);
  }

  last_finish_code_ = finish_code;
  publishIntentFeedback(static_cast<uint8_t>(active_task_intent_), finish_code);
  RCLCPP_WARN(logger_, "[TOP_HFSM_V2] abort intent=%u reason=%s",
              static_cast<unsigned>(active_task_intent_), reason);
}

bool TopHFSMNode::isSupportedIntent(uint8_t raw) {
  switch (static_cast<IntentType>(raw)) {
  case IntentType::IDLE:
  case IntentType::AUTO_INIT:
  case IntentType::TEST_SOLVE:
  case IntentType::TEST_CARTESIAN:
  case IntentType::AUTO_GRAB:
  case IntentType::AUTO_STORE:
  case IntentType::AUTO_GET:
  case IntentType::TELEOP_SERVO:
    return true;
  default:
    return false;
  }
}

} // namespace top_hfsm_v2

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(top_hfsm_v2::TopHFSMNode)
