#pragma once

/*
    作为动作通信服务端下属执行层对象
    接收服务端整理好的 GoalContext，调用 solve_core 完成规划
    对上提供 execute 接口，对下提供 MoveIt adapter 能力
*/

#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include "params_utils/param_utils.hpp"
#include "solve_core/moveit_adapter.hpp"
#include "solve_core/solve_core.hpp"

namespace arm_solve {

struct GoalContext;
struct ArmSolveConfig;

namespace solve_executor {

struct SolveExecutorConfig : public params_utils::MoveItResetConfig,
                             public params_utils::JointResetConfig {
  int late_init_delay_ms{10};
  int plan_min_interval_ms{0};

  void validate() const;
  std::string summary() const;
};

inline void SolveExecutorConfig::validate() const {
  params_utils::MoveItResetConfig::validate();
  params_utils::JointResetConfig::validate();
}

inline std::string SolveExecutorConfig::summary() const {
  std::ostringstream oss;
  oss << "=========\n";
  oss << " SolveExecutor Configuration\n\n";
  oss << params_utils::MoveItResetConfig::summary();
  oss << params_utils::JointResetConfig::summary();
  oss << " Planning:\n";
  oss << "   - late_init_delay_ms    : " << late_init_delay_ms << "\n";
  oss << "   - plan_min_interval_ms  : " << plan_min_interval_ms << "\n\n";
  oss << "=========\n";
  return oss.str();
}

class SolveExecutor : public solve_core::MoveItAdapter {
public:
  SolveExecutor(rclcpp::Node &host_node,
                const SolveExecutorConfig &config,
                const solve_core::SolveCoreConfig &solve_core_config);

  void lateInit(const std::shared_ptr<rclcpp::Node> &host_node);
  bool isReady() const;
  void stop();
  std::string planning_frame_id() const;

  bool execute(const arm_solve::GoalContext &ctx,
               solve_core::Trajectory &out_traj,
               std::string &err);

  std::shared_ptr<const moveit::core::RobotModel> robot_model() const override;
  const moveit::core::JointModelGroup *
  joint_model_group(const std::string &group_name) const override;
  std::string group_name() const override;
  std::string planning_frame() const override;
  std::string end_effector_link() const override;
  void set_start_state(const moveit::core::RobotState &state) override;

  std::optional<solve_core::Trajectory> plan_to_joint_target(
      const std::vector<std::string> &joint_names,
      const std::vector<double> &joint_values,
      const solve_core::PlannerConfigs &options) override;

  bool check_self_collision(const moveit::core::RobotState &state,
                            const std::string &group_name,
                            std::string &err) const override;

  bool time_parameterize_trajectory(const moveit::core::RobotState &start_state,
                                    const std::string &group_name,
                                    solve_core::Trajectory &traj,
                                    double velocity_scaling,
                                    double accel_scaling,
                                    std::string &err) const override;

private:
  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;
  SolveExecutorConfig config_;
  solve_core::SolveCoreConfig solve_core_config_;
  rclcpp::Time last_plan_time_;
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> psm_;
  std::shared_ptr<solve_core::MoveItAdapter> moveit_adapter_;
  std::unique_ptr<solve_core::SolveCore> solve_core_;
  mutable std::mutex exec_mutex_;

  bool planTrajectory(const arm_solve::GoalContext &ctx,
                      solve_core::Trajectory &out_traj,
                      std::string &err);
  void applyPlannerConfigs(const solve_core::PlannerConfigs &configs);
};

SolveExecutorConfig makeSolveExecutorConfig(const arm_solve::ArmSolveConfig &config);

} // namespace solve_executor
} // namespace arm_solve
