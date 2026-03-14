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
      .vision_mapped_vector("lift_cartesian")
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

TaskPlan make_test_solve() {
  const auto target0 =
      make_target(0.26235, -0.34444, 0.40285, 0.64923, 0.17886, 0.18983, 0.71447);
  const auto target1 =
      make_target(0.26484, -0.36076, 0.55316, 0.64923, 0.17886, 0.18983, 0.71447);
  geometry_msgs::msg::Vector3 target1_vec;
  // Use direction from target0 (current pose after first step) to target1
  target1_vec.x = target1.x - target0.x;
  target1_vec.y = target1.y - target0.y;
  target1_vec.z = target1.z - target0.z;
  return TaskBuilder(TaskId::TEST_SOLVE)
      .pose("move_normal_pose0", target0)
      .gripper("gripper_close", CLOSE)
      .delay("gripper_settle", 600)
      .vector("move_cartesian_pose1", target1_vec)
      .joints("move_home_joints", HOME)
      .build();
}

TaskPlan make_test_cartesian() {
  const auto target0 =
      make_target(0.44595, -0.086883, 0.61032, -0.17604, 0.69171, -0.39348, 0.57941);
  const auto target1 =
      make_target(0.54028, 0.19212, 0.27802, -0.17604, 0.69171, -0.39348, 0.57941);
  geometry_msgs::msg::Vector3 target1_vec;
  // Use direction from target0 (current pose after first step) to target1
  target1_vec.x = target1.x - target0.x;
  target1_vec.y = target1.y - target0.y;
  target1_vec.z = target1.z - target0.z;
  return TaskBuilder(TaskId::TEST_CARTESIAN)
      .pose("approach_normal", target0)
      .gripper("gripper_close", CLOSE)
      .delay("gripper_settle", 200)
      .vector("take_out_cartesian", target1_vec)
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
  case TaskId::TEST_SOLVE:
    return make_test_solve();
  case TaskId::TEST_CARTESIAN:
    return make_test_cartesian();
  case TaskId::IDLE:
  default:
    return std::nullopt;
  }
}

} // namespace task_orchestrator
