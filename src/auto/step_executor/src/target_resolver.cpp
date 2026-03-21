#include "step_executor/target_resolver.hpp"

#include <array>

namespace step_executor {
using namespace task_step_library;

bool TargetResolver::resolve(const ArmMoveSpec &input, const SharedData &data,
                             ArmMoveSpec &output, std::string &error) const {
  output = input;
  error.clear();

  switch (input.target_source) {
  case TargetSource::Fixed:
    return true;
  case TargetSource::SharedPose: {
    if (input.target_key != SharedKey::VisionPose) {
      error = "unsupported shared pose key";
      return false;
    }
    engineer_interfaces::msg::Pose pose;
    if (!data.get<SharedKey::VisionPose>(pose)) {
      error = "shared pose missing";
      return false;
    }
    output.pose = pose;
    return true;
  }
  case TargetSource::SharedVector: {
    if (input.target_key != SharedKey::VisionVector) {
      error = "unsupported shared vector key";
      return false;
    }
    geometry_msgs::msg::Vector3 vec;
    if (!data.get<SharedKey::VisionVector>(vec)) {
      error = "shared vector missing";
      return false;
    }
    output.vector = vec;
    return true;
  }
  case TargetSource::SlotMapped: {
    int slot = -1;
    if (!data.get<SharedKey::SelectedSlot>(slot)) {
      error = "selected slot missing";
      return false;
    }
    if (slot == 0) {
      output.plan_option = PlanOption::JOINTS;
      output.joints = std::array<float, 6>{{0.35f, -1.05f, -2.35f, 0.02f, 0.f, 0.f}};
      return true;
    }
    if (slot == 1) {
      output.plan_option = PlanOption::JOINTS;
      output.joints =
          std::array<float, 6>{{-0.35f, -1.05f, -2.35f, -0.02f, 0.f, 0.f}};
      return true;
    }
    error = "invalid selected slot";
    return false;
  }
  default:
    error = "unsupported arm target source";
    return false;
  }
}

} // namespace step_executor
