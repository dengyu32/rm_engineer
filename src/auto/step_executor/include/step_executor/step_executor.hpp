#pragma once

#include <cstddef>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "step_executor/capability_bridge.hpp"
#include "task_step_library/types.hpp"

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

private:
  void fail(task_step_library::TaskStatus status, const std::string &message);
  void enterNextStep();

  rclcpp::Logger logger_;
  std::shared_ptr<ICapabilityBridge> bridge_;

  task_step_library::TaskPlan plan_;
  std::size_t step_index_{0};
  bool step_entered_{false};
  int retries_left_{0};
  rclcpp::Time step_start_time_{0, 0, RCL_STEADY_TIME};

  bool running_{false};
  bool finished_{false};
  task_step_library::TaskResult report_;
};

} // namespace step_executor
