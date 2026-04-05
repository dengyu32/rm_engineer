#pragma once

#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>
#include <Eigen/Geometry>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

#include "types/types.hpp"
#include "robot_config/robot_config.hpp"

namespace collision
{
class SelfCollisionDetector;
}

/*
  执行器层
  - 向服务端节点ArmSolveServer提供执行接口
  - 调用规划器提供的plan接口进行路径规划
  - 延时初始化move_group，提供move_group接口
  - 对规划后的路径进行时间参数化
*/
namespace solve_executor
{
using namespace types;
// ============================================================================
//  PlanOption
// ----------------------------------------------------------------------------
//  - 规划方式枚举值定义
//  - NORMAL：使用move_group_interface请求plan实现，调用ompl规划
//  - CARTESIAN：直线路径规划
//  - JOINTS：关节空间规划
// ============================================================================
enum class PlanOption
{
  NORMAL = 0,
  CARTESIAN = 1,
  JOINTS = 2,
};

// ============================================================================
//  SolveRequest
// ----------------------------------------------------------------------------
//  - 规划请求结构体定义，数据类型包括：
//  - 规划方式选择
//  - 各规划方式对应的目标
//  - 当前关节状态
//  - 规划组配置
// ============================================================================
struct SolveRequest
{
  PlanOption option{ PlanOption::NORMAL };
  Pose target_pose{};
  Eigen::Vector3d target_vector{ { 0.0, 0.0, 0.0 } };
  double target_length{ 0.0 };
  std::vector<double> target_joints;
  JointState current_joints;
};

// ============================================================================
//  SolveExecutorConfig
// ----------------------------------------------------------------------------
//  - SolveExecutor节点参数加载，包括：
//  - MoveItResetConfig
//  - JointResetConfig
//  - SolveExecutorResetConfig
// ============================================================================
struct SolveExecutorConfig : public params_utils::MoveItResetConfig,
                             public params_utils::JointResetConfig,
                             public params_utils::SolveExecutorResetConfig
{
  static SolveExecutorConfig Load(rclcpp::Node& node)
  {
    SolveExecutorConfig cfg;
    params_utils::MoveItResetConfig::Load(node, cfg);
    params_utils::JointResetConfig::Load(node, cfg);
    params_utils::SolveExecutorResetConfig::Load(node, cfg);
    cfg.validate();
    return cfg;
  }

  void validate() const
  {
    params_utils::MoveItResetConfig::validate();
    params_utils::JointResetConfig::validate();
    params_utils::SolveExecutorResetConfig::validate();
  }

  std::string summary() const
  {
    std::ostringstream oss;
    oss << "=========\n";
    oss << " SolveExecutor Configuration\n\n";
    oss << params_utils::MoveItResetConfig::summary();
    oss << params_utils::JointResetConfig::summary();
    oss << params_utils::SolveExecutorResetConfig::summary();
    oss << "=========\n";
    return oss.str();
  }
};

// ============================================================================
//  SolveExecutor
// ----------------------------------------------------------------------------
//  - 功能实现类
// ============================================================================
class SolveExecutor
{
public:
  // 构造函数，允许配置
  explicit SolveExecutor(rclcpp::Node& node);

  // 终止move_group_interface的规划请求
  void stop();

  // 节点程序运行
  bool execute(const SolveRequest& req, solve_executor::Trajectory& out_traj, std::string& err);

private:
  // 判断延迟初始化是否成功
  bool isReady() const;
  bool ensureInitialized(std::string& err);

  // 规划接口
  bool plan(const SolveRequest& req, solve_executor::Trajectory& out_traj, std::string& err);
  bool plan_normal(const SolveRequest& req, solve_executor::Trajectory& out_traj, std::string& err,
                   const ::collision::SelfCollisionDetector* self_collision_detector = nullptr);
  bool plan_cartesian(const SolveRequest& req, solve_executor::Trajectory& out_traj, std::string& err,
                      const ::collision::SelfCollisionDetector* self_collision_detector = nullptr);
  bool plan_joints(const SolveRequest& req, solve_executor::Trajectory& out_traj, std::string& err,
                   const ::collision::SelfCollisionDetector* self_collision_detector = nullptr);

  // 碰撞检测，在外部单独实现
  // bool validateTrajectoryCollision(const robot_trajectory::RobotTrajectory& robot_traj, const std::string& group_name,
  //                                  std::string& err) const;
  // 时间参数化
  bool timeParameterizeTrajectory(solve_executor::Trajectory& traj, std::string& err) const;
  void parameterize_time_from_start(solve_executor::Trajectory &traj, double velocity_scaling);

  rclcpp::Node& node_;
  rclcpp::Clock::SharedPtr clock_;
  SolveExecutorConfig config_;
  rclcpp::Time last_plan_time_;

  std::shared_ptr<rclcpp::Node> node_handle_;
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> psm_;
  moveit::core::RobotModelConstPtr robot_model_;
  std::unique_ptr<moveit::core::RobotState> current_state_;

  mutable std::mutex exec_mutex_;
  mutable std::mutex init_mutex_;
};

}  // namespace solve_executor
