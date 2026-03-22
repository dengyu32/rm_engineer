#include "task_orchestrator/task_orchestrator.hpp"
#include "task_orchestrator/task_builder.hpp"
#include "task_orchestrator/robot_state.hpp"

namespace task_orchestrator {
using namespace task_step_library;

namespace {

inline engineer_interfaces::msg::Pose make_target(double x, double y, double z,
                                                     double qx, double qy,
                                                     double qz, double qw) {
  engineer_interfaces::msg::Pose t;
  t.x = x;
  t.y = y;
  t.z = z;
  t.qx = qx;
  t.qy = qy;
  t.qz = qz;
  t.qw = qw;
  return t;
}

constexpr double kCartesianTargetLengthM = 0.03;

TaskPlan make_auto_init() {
  return TaskBuilder(TaskId::AUTO_INIT)
      .gripper("gripper_open", OPEN)
      .joints("move_home_joints", HOME)
      .build();
}

TaskPlan make_auto_grab() {
  return TaskBuilder(TaskId::AUTO_GRAB)
      .vision("vision_detect")
      .vision_mapped_pose("approach_to_grab")
      .gripper("gripper_close", CLOSE)
      .delay("gripper_settle", 600)
      .vision_mapped_vector("lift_cartesian", kCartesianTargetLengthM)
      .joints("move_home", HOME)
      .build();
}

TaskPlan make_auto_store() {
  return TaskBuilder(TaskId::AUTO_STORE)
      .slot("select_slot", PUT)
      .slot_mapped_joints("move_to_place_slot")
      .gripper("auto_store_gripper_open", OPEN)
      .delay("auto_store_release_settle", 600)
      .slot("lock_slot", LOCK)
      .joints("auto_store_move_home", HOME)
      .build();
}

TaskPlan make_auto_get() {
  return TaskBuilder(TaskId::AUTO_GET)
      .slot("select_slot", TAKE)
      .slot_mapped_joints("move_to_slot")
      .slot("unlock_slot", UNLOCK)
      .gripper("gripper_close", CLOSE)
      .delay("gripper_settle", 600)
      .joints("move_home", HOME)
      .build();
}

// 除了 G1 G4 其他都加负号
TaskPlan make_fixed_grab() {
  const std::array<float,6> fixed_joints_1{
    -0.0563895777f,
    0.183680534f,
    -1.25949967f,
    0.603695154f,
    -0.151254654f,
    -0.378172457f
  };

  const std::array<float,6> fixed_joints_2{
    -0.00333404541f,
    -0.364118576f,
    -0.594209731f,
    0.612900198f,
    -0.221446991f,
    -0.376638293f
  };

  const std::array<float,6> fixed_joints_3{
    -0.324277222f,
    -0.578888893f,
    -0.0442333445f,
    0.747139871f,
    -0.393873215f,
    -0.754043639f
  };
  
  return TaskBuilder(TaskId::FIXED_GRAB)
      .gripper("open_gripper", OPEN)
      .delay("gripper_settle", 300)
      .joints("move_fixed_joints_1", fixed_joints_1)
      .joints("move_fixed_joints_2", fixed_joints_2)
      .joints("move_fixed_joints_3", fixed_joints_3)
      .joints("move_home", HOME)
      .build();
}

TaskPlan make_test_solve() {
  const auto target0 =
      make_target(-0.52816, 0.07035, 0.57682, -0.35088, -0.59813, 0.36457, 0.62146);
  const auto target1 =
      make_target(-0.63131, -0.052031, 0.67485, -0.18348, -0.64249, 0.27727, 0.6904);
  geometry_msgs::msg::Vector3 target1_vec;
  // Use direction from target0 (current pose after first step) to target1
  target1_vec.x = target1.x - target0.x;
  target1_vec.y = target1.y - target0.y;
  target1_vec.z = target1.z - target0.z;
  return TaskBuilder(TaskId::TEST_SOLVE)
      .pose("move_normal_pose0", target0)
      .gripper("gripper_close", CLOSE)
      .delay("gripper_settle", 600)
      .vector("move_cartesian_pose1", target1_vec, kCartesianTargetLengthM)
      .joints("move_home_joints", HOME)
      .build();
}

TaskPlan make_test_cartesian() {
  const auto target0 =
      make_target(-0.63131, -0.052031, 0.67485, -0.18348, -0.64249, 0.27727, 0.6904);
  const auto target1 =
      make_target(-0.52816, 0.07035, 0.57682, -0.35088, -0.59813, 0.36457, 0.62146);
  geometry_msgs::msg::Vector3 target1_vec;
  // Use direction from target0 (current pose after first step) to target1
  target1_vec.x = target1.x - target0.x;
  target1_vec.y = target1.y - target0.y;
  target1_vec.z = target1.z - target0.z;
  return TaskBuilder(TaskId::TEST_CARTESIAN)
      .pose("approach_normal", target0)
      .gripper("gripper_close", CLOSE)
      .delay("gripper_settle", 200)
      .vector("take_out_cartesian", target1_vec, kCartesianTargetLengthM)
      .gripper("gripper_open", OPEN)
      .build();
}

} // namespace

std::optional<TaskPlan>
TaskOrchestrator::plan(const TaskRequest &request) const {
  switch (request.task_id) {
  case TaskId::AUTO_INIT:
    return make_auto_init();
  case TaskId::AUTO_GRAB:
    return make_auto_grab();
  case TaskId::AUTO_STORE:
    return make_auto_store();
  case TaskId::AUTO_GET:
    return make_auto_get();
  case TaskId::FIXED_GRAB:
    return make_fixed_grab();
  case TaskId::TEST_SOLVE:
    return make_test_solve();
  case TaskId::TEST_CARTESIAN:
    return make_test_cartesian();
  case TaskId::IDLE:
  default:
    return std::nullopt;
  }
}
 
// 扩展意见

} // namespace task_orchestrator
