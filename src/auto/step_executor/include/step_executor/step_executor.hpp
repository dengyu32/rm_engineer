#pragma once

#include <cstddef>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "step_executor/capability_bridge.hpp"
#include "step_executor/target_resolver.hpp"
#include "shared_data/context.hpp"
#include "task_step_library/task.hpp"

namespace step_executor {

class StepExecutor {
public:
  explicit StepExecutor(rclcpp::Logger logger,
                        std::shared_ptr<ICapabilityBridge> bridge =
                            std::make_shared<NoopCapabilityBridge>());

  void start(const task_step_library::TaskPlan &plan);
  void tick(const rclcpp::Time &now);
  void cancel();
  void reset();

  bool isRunning() const;
  bool isFinished() const;
  task_step_library::TaskResult report() const;
  task_step_library::TaskId activeTaskId() const;
  std::size_t currentStepIndex() const;
  std::size_t totalSteps() const;
  std::string currentStepLabel() const;

private:
  bool deriveStepFromSharedData(const task_step_library::Step &input,
                                task_step_library::Step &derived,
                                std::string &error) const;
  void applyStepResult(const task_step_library::StepResult &result);
  void fail(task_step_library::TaskStatus status, const std::string &message);
  void enterNextStep();

  rclcpp::Logger logger_;
  std::shared_ptr<ICapabilityBridge> bridge_;
  TargetResolver resolver_{};

  task_step_library::TaskPlan plan_;
  std::size_t step_index_{0};
  bool step_entered_{false};
  int retries_left_{0};
  rclcpp::Time step_start_time_{0, 0, RCL_STEADY_TIME};

  bool running_{false};
  bool finished_{false};
  task_step_library::TaskResult report_;
  task_step_library::SharedData data_{};
};

} // namespace step_executor
