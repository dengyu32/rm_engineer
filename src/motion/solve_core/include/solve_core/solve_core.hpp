#pragma once

#include <memory>
#include <optional>
#include <string>

#include "solve_core/adapter.hpp"
#include "solve_core/moveit_adapter.hpp"

namespace solve_core {

enum class PlanOption {
  NORMAL = 0,
  CARTESIAN = 1,
  JOINTS = 2,
};

struct SolveRequest {
  PlanOption option{PlanOption::NORMAL};
  Pose target_pose{};
  std::array<double, 3> target_vector{{0.0, 0.0, 0.0}};
  double target_length{0.0};
  std::vector<double> target_joints;
  JointState current_joints;
  std::string group_name;
  std::string ee_link;
};

class SolveCore {
public:

  explicit SolveCore(std::shared_ptr<MoveItAdapter> adapter,
                     const SolveCoreConfig &config = SolveCoreConfig{});

  std::optional<SolveResponse> plan(const SolveRequest &req, std::string &err);

private:
  std::shared_ptr<MoveItAdapter> adapter_;
  SolveCoreConfig config_;

  std::optional<SolveResponse> plan_normal(const SolveRequest &req, std::string &err);
  std::optional<SolveResponse> plan_cartesian(const SolveRequest &req, std::string &err);
  std::optional<SolveResponse> plan_joints(const SolveRequest &req, std::string &err);

};

} // namespace solve_core
