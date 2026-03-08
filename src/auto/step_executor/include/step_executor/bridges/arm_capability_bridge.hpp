#pragma once

#include <string>

#include "arm_solve_client/arm_solve_client.hpp"
#include "step_executor/capability_bridge.hpp"

namespace step_executor {

class ArmCapabilityBridge {
public:
  explicit ArmCapabilityBridge(rclcpp::Node &node);

  BridgeResult runArmStep(const task_step_library::Step &step,
                          task_step_library::StepResult *out_result);
  void cancel();
  const char *lastError() const { return last_error_.c_str(); }

private:
  engineer_auto::arm_solve_client::ArmSolveClient client_;
  std::string last_error_;
};

} // namespace step_executor
