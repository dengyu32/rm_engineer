#pragma once

#include <string>

#include "top_hfsm_v2/types/command_types.hpp"

namespace top_hfsm_v2 {

class ArmExecutor {
public:
  virtual ~ArmExecutor() = default;

  virtual ArmCommandStatus execute_arm_command(const ArmCommandSpec &command) = 0;
  virtual void sendCancel() = 0;
  virtual std::string lastErrorMessage() const = 0;
};

class GripperExecutor {
public:
  virtual ~GripperExecutor() = default;

  virtual void setGripperPosition(GripperCommandType command) = 0; //set_gripper_position 
};

class ServoExecutor {
public:
  virtual ~ServoExecutor() = default;

  virtual ServoCommandResult startServo() = 0;
  virtual ServoCommandResult pauseServo() = 0;
};

} // namespace top_hfsm_v2
