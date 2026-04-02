#pragma once

#include <rclcpp/rclcpp.hpp>

#include <sstream>
#include <stdexcept>
#include <string>

#include "params_utils/core/params.hpp"

namespace params_utils
{

struct SolveExecutorResetConfig
{
  int late_init_delay_ms{ 10 };   // 延迟初始化时间
  int plan_min_interval_ms{ 0 };  // 两次规划最短时间间隔

  double goal_position_tolerance{ 1e-3 };
  double goal_orientation_tolerance{ 1e-3 };
  double planning_time{ 2.0 };
  int num_planning_attempts{ 5 };

  double max_velocity_scaling{ 1.0 };
  double max_acc_scaling{ 1.0 };

  bool use_vision_target_vector{ true };
  double sample_step_m{ 0.005 };          // 每步位移（米）
  double alignment_dot_threshold{ 0.98 };
  std::string reference_link{ "base_link" };
  int max_contacts{ 8 };
  int max_contacts_per_pair{ 1 };


  static void Load(rclcpp::Node& node, SolveExecutorResetConfig& cfg)
  {
    using params_utils::detail::declare_get;
    using params_utils::detail::declare_get_checked;
    declare_get_checked(
        node, "late_init_delay_ms", cfg.late_init_delay_ms, [](const double v) { return v > 0; },
        "must be a positive number");
    declare_get_checked(
        node, "plan_min_interval_ms", cfg.plan_min_interval_ms, [](const double v) { return v >= 0; },
        "must be a non-negative number");
    declare_get_checked(
        node, "goal_position_tolerance", cfg.goal_position_tolerance, [](const double v) { return v >= 0; },
        "must be a non-negative number");
    declare_get_checked(
        node, "goal_orientation_tolerance", cfg.goal_orientation_tolerance, [](const double v) { return v >= 0; },
        "must be a non-negative number");
    declare_get_checked(
        node, "planning_time", cfg.planning_time, [](const double v) { return v > 0; }, "must be a positive number");
    declare_get_checked(
        node, "num_planning_attempts", cfg.num_planning_attempts, [](const double v) { return v > 0; },
        "must be a positive number");
    declare_get_checked(
        node, "max_velocity_scaling", cfg.max_velocity_scaling, [](const double v) { return v > 0; },
        "must be a positive number");
    declare_get_checked(
        node, "max_acc_scaling", cfg.max_acc_scaling, [](const double v) { return v > 0; }, "must be a positive number");
    declare_get(node, "use_vision_target_vector", cfg.use_vision_target_vector);
    declare_get_checked(
        node, "sample_step_m", cfg.sample_step_m, [](const double v) { return v > 0; }, "must be a positive number");
    declare_get_checked(node, "alignment_dot_threshold", cfg.alignment_dot_threshold,
                        [](const double v) { return v >= 0.0 && v <= 1.0; }, "must be in [0, 1]");
    declare_get_checked(node, "reference_link", cfg.reference_link, [](const std::string& v) { return !v.empty(); },
                        "must be a non-empty string");
    declare_get_checked(
        node, "max_contacts", cfg.max_contacts, [](const double v) { return v > 0; }, "must be a positive number");
    declare_get_checked(node, "max_contacts_per_pair", cfg.max_contacts_per_pair, [](const double v) { return v > 0; },
                        "must be a positive number");
    cfg.validate();
  }

  void validate() const
  {
  }

  std::string summary() const
  {
    std::ostringstream oss;
    oss << " MoveIt:\n";
    oss << "   - late_init_delay_ms              : " << late_init_delay_ms << "\n";
    oss << "   - plan_min_interval_ms            : " << plan_min_interval_ms << "\n";
    oss << "   - goal_position_tolerance         : " << goal_position_tolerance << "\n";
    oss << "   - goal_orientation_tolerance      : " << goal_orientation_tolerance << "\n";
    oss << "   - planning_time                   : " << planning_time << "\n";
    oss << "   - num_planning_attempts           : " << num_planning_attempts << "\n";
    oss << "   - max_velocity_scaling            : " << max_velocity_scaling << "\n";
    oss << "   - max_acc_scaling                 : " << max_acc_scaling << "\n";
    oss << "   - use_vision_target_vector        : " << use_vision_target_vector << "\n";
    oss << "   - sample_step_m                   : " << sample_step_m << "\n";
    oss << "   - alignment_dot_threshold         : " << alignment_dot_threshold << "\n";
    oss << "   - reference_link                  : " << reference_link << "\n";
    oss << "   - max_contacts                    : " << max_contacts << "\n";
    oss << "   - max_contacts_per_pair           : " << max_contacts_per_pair << "\n";
    return oss.str();
  }
};

}  // namespace params_utils
