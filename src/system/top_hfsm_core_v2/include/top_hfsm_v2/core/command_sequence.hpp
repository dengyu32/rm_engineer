#pragma once

#include <cstddef>
#include <functional>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "top_hfsm_v2/core/async_dispatcher.hpp"
#include "top_hfsm_v2/core/runtime_context.hpp"
#include "top_hfsm_v2/types/command_types.hpp"

namespace top_hfsm_v2 {

using GuardEvaluator =
    std::function<bool(int guard_id, const RuntimeContext &context)>;

class CommandSequence {
public:
  CommandSequence();

  void setSteps(const std::vector<CommandStep> &steps) { steps_ = steps; }
  void setGuardEvaluator(GuardEvaluator evaluator);
  void reset();

  void update(RuntimeContext &context, const rclcpp::Time &now);

  bool isFinished() const { return is_finished_; }
  TaskStatus result() const { return result_status_; }
  const std::string &reason() const { return result_message_; }

private:
  static bool defaultGuard(int, const RuntimeContext &);

  void advanceToNextStep();
  void handleStepFailure(RuntimeContext &context, const std::string &message,
                         TaskStatus status, const rclcpp::Time &now);
  void handleArmMove(const CommandStep &step, RuntimeContext &context,
                     const rclcpp::Time &now);
  void handleGripper(const CommandStep &step, RuntimeContext &context);
  void handleDelay(const CommandStep &step, const rclcpp::Time &now);
  void handleGuard(const CommandStep &step, RuntimeContext &context);
  void handleServo(const CommandStep &step, RuntimeContext &context,
                   bool start, const rclcpp::Time &now);

  std::vector<CommandStep> steps_;
  GuardEvaluator guard_evaluator_;

  std::size_t current_step_index_{0};
  bool current_step_entered_{false};
  int remaining_retries_{0};
  bool is_finished_{false};
  TaskStatus result_status_{TaskStatus::Success};
  std::string result_message_;
  rclcpp::Time current_step_start_time_{0, 0, RCL_STEADY_TIME};
};

} // namespace top_hfsm_v2
