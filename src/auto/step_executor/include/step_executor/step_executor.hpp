#pragma once

#include <cstddef>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "step_executor/types/capability_bridge.hpp"
#include "step_executor/types/command.hpp"
#include "step_executor/types/context.hpp"
#include "step_executor/types/execute_result.hpp"
#include "step_executor/types/task.hpp"

namespace step_executor {

class StepExecutor {
public:
  explicit StepExecutor(rclcpp::Logger logger,
                        std::shared_ptr<ICapabilityBridge> bridge =
                            std::make_shared<NoopCapabilityBridge>());

  void start(const TaskPlan &plan);
  void tick(const rclcpp::Time &now);
  void cancel();
  void reset();

  bool isRunning() const;
  bool isFinished() const;
  TaskResult report() const;
  TaskId activeTaskId() const;
  std::size_t currentStepIndex() const;
  std::size_t totalSteps() const;
  std::string currentStepLabel() const;

private:
  bool applyBindings(const Step &step, Command &cmd, std::string &error) const;
  bool applyOutputs(const Step &step, const ExecuteResult &result, std::string &error);
  void fail(TaskStatus status, const std::string &message);
  void enterNextStep();

  rclcpp::Logger logger_;
  std::shared_ptr<ICapabilityBridge> bridge_;
  TaskPlan plan_{};
  std::size_t step_index_{0};
  bool step_entered_{false};
  int retries_left_{0};
  rclcpp::Time step_start_time_{0, 0, RCL_STEADY_TIME};

  bool running_{false};
  bool finished_{false};
  TaskResult report_{};
  ContextStore context_{};
};

} // namespace step_executor
