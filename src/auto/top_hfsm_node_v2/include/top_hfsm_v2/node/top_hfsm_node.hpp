#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <engineer_interfaces/msg/intent.hpp>

#include "error_code_utils/error_bus.hpp"
#include "top_hfsm_v2/config.hpp"
#include "top_hfsm_v2/core/command_sequence.hpp"
#include "top_hfsm_v2/core/runtime_context.hpp"
#include "top_hfsm_v2/executors/arm_solve_client.hpp"
#include "top_hfsm_v2/executors/gripper_control_node.hpp"

namespace top_hfsm_v2 {

struct NodeConfig {
  std::string intent_cmd_topic{"/hfsm/intent_commands"};
  std::string intent_fb_topic{"/hfsm/intent_feedback"};
  int hfsm_update_period_ms{20};

  ArmSolveClientConfig arm_solve_client;
  GripperConfig gripper;

  static NodeConfig Load(rclcpp::Node &node);
  void validate() const;
};

enum class IntentType : uint8_t {
  IDLE = 0,
  AUTO_INIT = 1,
  TEST_SOLVE = 2,
  TEST_CARTESIAN = 3,
  AUTO_GRAB = 4,
  AUTO_STORE = 5,
  AUTO_GET = 6,
};

enum class FinishCode : uint8_t {
  Running = 0,
  Finished = 1,
  Aborted = 2,
};

enum class ActiveMode : uint8_t {
  Idle = 0,
  CommandSequence = 1,
};

class TopHFSMNode : public rclcpp::Node {
public:
  explicit TopHFSMNode(const rclcpp::NodeOptions &options);
  ~TopHFSMNode() override;

private:
  void intentCallback(engineer_interfaces::msg::Intent::ConstSharedPtr msg);
  void tick();
  void publishIntentFeedback(uint8_t intent_id, FinishCode finish_code);
  void handleIntentTransition(IntentType next_intent);
  void beginSequenceForIntent(IntentType intent);
  void enterIdle();
  void abortCurrentIntent(FinishCode finish_code, const char *reason);

  static bool isSupportedIntent(uint8_t raw);

  NodeConfig config_;
  rclcpp::Logger logger_;

  executors::ArmSolveClient arm_solve_client_;
  executors::GripperControlNode gripper_control_node_;
  RuntimeContext runtime_context_;
  CommandSequence command_sequence_;

  std::atomic<IntentType> latest_intent_{IntentType::IDLE};
  IntentType applied_intent_{IntentType::IDLE};
  IntentType active_task_intent_{IntentType::IDLE};
  ActiveMode active_mode_{ActiveMode::Idle};
  FinishCode last_finish_code_{FinishCode::Running};

  rclcpp::Subscription<engineer_interfaces::msg::Intent>::SharedPtr intent_sub_;
  rclcpp::Publisher<engineer_interfaces::msg::Intent>::SharedPtr intent_fb_pub_;
  rclcpp::TimerBase::SharedPtr update_timer_;

  std::shared_ptr<error_code_utils::ErrorBus> error_bus_;
};

} // namespace top_hfsm_v2
