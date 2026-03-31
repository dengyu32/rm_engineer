#pragma once

#include <cstddef>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "auto_library/command.hpp"
#include "auto_library/context.hpp"
#include "auto_library/execute_result.hpp"
#include "auto_library/task.hpp"
#include "step_executor/registry_bridge.hpp"

namespace step_executor {

// ============================================================================
//  StepExecutor
// ----------------------------------------------------------------------------
//  - 执行 TaskPlan（线性）
//  - 绑定 inputs -> params
//  - 调用 capability，并写回 outputs
//  - 负责超时/重试/失败策略
// ============================================================================

class StepExecutor {
public:
  explicit StepExecutor(rclcpp::Logger logger,
                        std::shared_ptr<ICapabilityBridge> bridge);

  void start(const core::TaskPlan &plan);
  void tick(const rclcpp::Time &now);
  void cancel();
  void reset();

  bool isRunning() const;
  bool isFinished() const;
  core::TaskResult report() const;
  core::TaskId activeTaskId() const;
  std::size_t currentStepIndex() const;
  std::size_t totalSteps() const;
  std::string currentStepLabel() const;

private:
  bool applyBindings(const core::Step &step, core::Command &cmd, std::string &error) const;
  bool applyOutputs(const core::Step &step,
                    const core::ExecuteResult &result,
                    std::string &error);
  void fail(core::TaskStatus status, const std::string &message);
  void enterNextStep();

  rclcpp::Logger logger_;
  std::shared_ptr<ICapabilityBridge> bridge_;
  core::TaskPlan plan_{};
  std::size_t step_index_{0};
  bool step_entered_{false};
  int retries_left_{0};
  rclcpp::Time step_start_time_{0, 0, RCL_STEADY_TIME};
  bool post_delay_active_{false};
  rclcpp::Time post_delay_start_time_{0, 0, RCL_STEADY_TIME};

  bool running_{false};
  bool finished_{false};
  core::TaskResult report_{};
  core::ContextStore context_{};
};

} // namespace step_executor
