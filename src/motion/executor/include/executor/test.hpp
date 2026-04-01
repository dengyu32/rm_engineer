#pragma once

#include <Eigen/Geometry>
#include <moveit/robot_model/link_model.h>
#include <moveit/robot_state/robot_state.h>

#include <string>

namespace solve_executor
{

bool set_vector_with_current(const moveit::core::RobotState& robot_state, const moveit::core::LinkModel* ref_link,
                             const moveit::core::LinkModel* ee_link, Eigen::Vector3d& target_vector, std::string& err);

}  // namespace solve_executor
