#pragma once

#include <string>

#include "arm_solve_client/arm_solve_client.hpp"
#include "step_executor/types/capability_bridge.hpp"

namespace step_executor {

class ArmCapabilityBridge {
public:
  explicit ArmCapabilityBridge(rclcpp::Node &node);

  ExecuteResult run(const Command &cmd);
  void cancel();
  const char *lastError() const { return last_error_.c_str(); }

private:
  engineer_auto::arm_solve_client::ArmSolveClient client_;
  std::string last_error_;
};

} // namespace step_executor
