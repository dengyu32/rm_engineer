#include "task_orchestrator/task_orchestrator.hpp"
#include "task_orchestrator/task_builder.hpp"
#include "task_orchestrator/protocol.hpp"

#include <engineer_interfaces/msg/pose.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include <array>
#include <cstddef>
#include <string>

#include "auto_library/context.hpp"
#include "auto_library/step.hpp"

namespace task_orchestrator {
namespace {

using step_executor::Binding;
using step_executor::ContextKey;
using step_executor::ContextScope;
using step_executor::Step;
namespace protocol = task_orchestrator::protocol;

inline ContextKey taskKey(const char *name) {
  return ContextKey{std::string(name), ContextScope::Task};
}

inline Binding bindTask(const char *key, const char *param) {
  return Binding{taskKey(key), std::string(param)};
}

inline Binding bindTaskIndexToJoints(const char *key, const char *param,
                                     const std::array<float, 6> *table,
                                     size_t table_size) {
  Binding binding{taskKey(key), std::string(param)};
  binding.op = step_executor::BindingOp::IndexToJointsTable;
  binding.joints_table = table;
  binding.joints_table_size = table_size;
  return binding;
}

inline Step makeStep(const char *id, const char *kind, int timeout_ms = 0, int retries = 0) {
  Step step;
  step.id = id;
  step.label = id;
  step.command.kind = kind;
  step.timeout_ms = timeout_ms;
  step.max_retries = retries;
  return step;
}

inline std::array<double, 7> toPoseArray(const engineer_interfaces::msg::Pose &pose) {
  return {pose.x, pose.y, pose.z, pose.qx, pose.qy, pose.qz, pose.qw};
}

inline std::array<double, 3> toVectorArray(const geometry_msgs::msg::Vector3 &vec) {
  return {vec.x, vec.y, vec.z};
}

Step make_delay(const char *id, int delay_ms) {
  Step step = makeStep(id, "");
  step.type = step_executor::StepType::Control;
  step.control.kind = step_executor::ControlKind::Delay;
  step.control.delay_ms = delay_ms;
  return step;
}

Step make_vision_detect(const char *id, int timeout_ms = 3000, int retries = 0) {
  Step step = makeStep(id, protocol::kVisionKind, timeout_ms, retries);
  step.outputs.push_back(taskKey(protocol::kVisionPose));
  step.outputs.push_back(taskKey(protocol::kVisionVector));
  return step;
}

Step make_arm_move_pose(const char *id, const engineer_interfaces::msg::Pose &target,
                        int timeout_ms = 8000, int retries = 1) {
  Step step = makeStep(id, protocol::kArmMoveKind, timeout_ms, retries);
  step.command.params["target_pose"] = toPoseArray(target);
  return step;
}

Step make_arm_move_joints(const char *id, const std::array<float, 6> &target,
                          int timeout_ms = 8000, int retries = 1) {
  Step step = makeStep(id, protocol::kArmMoveKind, timeout_ms, retries);
  step.command.params["target_joints"] = target;
  return step;
}

Step make_arm_move_vector(const char *id, const geometry_msgs::msg::Vector3 &target,
                          int timeout_ms = 8000, int retries = 1) {
  Step step = makeStep(id, protocol::kArmMoveKind, timeout_ms, retries);
  step.command.params["target_vector"] = toVectorArray(target);
  return step;
}

Step make_arm_move_from_key(const char *id, const char *key_name, const char *param_name,
                            int timeout_ms = 8000, int retries = 1) {
  Step step = makeStep(id, protocol::kArmMoveKind, timeout_ms, retries);
  step.inputs.push_back(taskKey(key_name));
  step.bindings.push_back(bindTask(key_name, param_name));
  return step;
}

Step make_arm_move_from_slot(const char *id, const char *key_name, const char *param_name,
                             const std::array<float, 6> *table, size_t table_size,
                             int timeout_ms = 8000, int retries = 1) {
  Step step = makeStep(id, protocol::kArmMoveKind, timeout_ms, retries);
  step.inputs.push_back(taskKey(key_name));
  step.bindings.push_back(bindTaskIndexToJoints(key_name, param_name, table, table_size));
  return step;
}

Step make_gripper(const char *id, const char *action) {
  Step step = makeStep(id, protocol::kGripperKind);
  step.command.params["action"] = std::string(action);
  return step;
}

Step make_slot_select(const char *id, const char *strategy) {
  Step step = makeStep(id, protocol::kSlotSelectKind);
  step.command.params["strategy"] = std::string(strategy);
  step.outputs.push_back(taskKey(protocol::kSlotId));
  return step;
}

Step make_slot_lock(const char *id, const char *slot_key) {
  Step step = makeStep(id, protocol::kSlotLockKind);
  step.inputs.push_back(taskKey(slot_key));
  step.bindings.push_back(bindTask(slot_key, "slot_id"));
  return step;
}

Step make_slot_unlock(const char *id, const char *slot_key) {
  Step step = makeStep(id, protocol::kSlotUnlockKind);
  step.inputs.push_back(taskKey(slot_key));
  step.bindings.push_back(bindTask(slot_key, "slot_id"));
  return step;
}

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

step_executor::TaskPlan make_auto_init() {
  return TaskBuilder(TaskId::AUTO_INIT)
      .add(make_gripper("gripper_open", "open"))
      .add(make_arm_move_joints("move_home_joints", task_orchestrator::preset::HOME))
      .build();
}

step_executor::TaskPlan make_auto_grab() {
  return TaskBuilder(TaskId::AUTO_GRAB)
      .add(make_vision_detect("vision_detect"))
      .add(make_arm_move_from_key("approach_to_grab",
                                  protocol::kVisionPose,
                                  "target_pose"))
      .add(make_gripper("gripper_close", "close"))
      .add(make_delay("gripper_settle", 600))
      .add(make_arm_move_from_key("lift_cartesian",
                                  protocol::kVisionVector,
                                  "target_vector"))
      .add(make_arm_move_joints("move_home", task_orchestrator::preset::HOME))
      .build();
}

step_executor::TaskPlan make_auto_store() {
  return TaskBuilder(TaskId::AUTO_STORE)
      .add(make_slot_select("select_slot", "put"))
      .add(make_arm_move_from_slot("move_to_place_slot",
                                   protocol::kSlotId,
                                   "target_joints",
                                   task_orchestrator::preset::SLOTS,
                                   std::size(task_orchestrator::preset::SLOTS)))
      .add(make_gripper("gripper_open", "open"))
      .add(make_delay("release_settle", 600))
      .add(make_slot_lock("lock_slot", protocol::kSlotId))
      .add(make_arm_move_joints("move_home", task_orchestrator::preset::HOME))
      .build();
}

step_executor::TaskPlan make_auto_get() {
  return TaskBuilder(TaskId::AUTO_GET)
      .add(make_slot_select("select_slot", "take"))
      .add(make_arm_move_from_slot("move_to_slot",
                                   protocol::kSlotId,
                                   "target_joints",
                                   task_orchestrator::preset::SLOTS,
                                   std::size(task_orchestrator::preset::SLOTS)))
      .add(make_slot_unlock("unlock_slot", protocol::kSlotId))
      .add(make_gripper("gripper_close", "close"))
      .add(make_delay("gripper_settle", 600))
      .add(make_arm_move_joints("move_home", task_orchestrator::preset::HOME))
      .build();
}

step_executor::TaskPlan make_fixed_grab() {
  const std::array<float, 6> fixed_joints_1{
      -0.0563895777f,
      0.183680534f,
      -1.25949967f,
      0.603695154f,
      -0.151254654f,
      -0.378172457f};

  const std::array<float, 6> fixed_joints_2{
      -0.00333404541f,
      -0.364118576f,
      -0.594209731f,
      0.612900198f,
      -0.221446991f,
      -0.376638293f};

  const std::array<float, 6> fixed_joints_3{
      -0.324277222f,
      -0.578888893f,
      -0.0442333445f,
      0.747139871f,
      -0.393873215f,
      -0.754043639f};

  return TaskBuilder(TaskId::FIXED_GRAB)
      .add(make_gripper("open_gripper", "open"))
      .add(make_delay("gripper_settle", 300))
      .add(make_arm_move_joints("move_fixed_joints_1", fixed_joints_1))
      .add(make_arm_move_joints("move_fixed_joints_2", fixed_joints_2))
      .add(make_arm_move_joints("move_fixed_joints_3", fixed_joints_3))
      .add(make_arm_move_joints("move_home", task_orchestrator::preset::HOME))
      .build();
}

step_executor::TaskPlan make_test_solve() {
  const auto target0 =
      make_target(-0.52816, 0.07035, 0.57682, -0.35088, -0.59813, 0.36457, 0.62146);
  const auto target1 =
      make_target(-0.63131, -0.052031, 0.67485, -0.18348, -0.64249, 0.27727, 0.6904);
  geometry_msgs::msg::Vector3 target1_vec;
  target1_vec.x = target1.x - target0.x;
  target1_vec.y = target1.y - target0.y;
  target1_vec.z = target1.z - target0.z;
  return TaskBuilder(TaskId::TEST_SOLVE)
      .add(make_arm_move_pose("move_normal_pose0", target0))
      .add(make_gripper("gripper_close", "close"))
      .add(make_delay("gripper_settle", 600))
      .add(make_arm_move_vector("move_cartesian_pose1", target1_vec))
      .add(make_arm_move_joints("move_home_joints", task_orchestrator::preset::HOME))
      .build();
}

step_executor::TaskPlan make_test_cartesian() {
  const auto target0 =
      make_target(-0.63131, -0.052031, 0.67485, -0.18348, -0.64249, 0.27727, 0.6904);
  const auto target1 =
      make_target(-0.52816, 0.07035, 0.57682, -0.35088, -0.59813, 0.36457, 0.62146);
  geometry_msgs::msg::Vector3 target1_vec;
  target1_vec.x = target1.x - target0.x;
  target1_vec.y = target1.y - target0.y;
  target1_vec.z = target1.z - target0.z;
  return TaskBuilder(TaskId::TEST_CARTESIAN)
      .add(make_arm_move_pose("approach_normal", target0))
      .add(make_gripper("gripper_close", "close"))
      .add(make_delay("gripper_settle", 200))
      .add(make_arm_move_vector("take_out_cartesian", target1_vec))
      .add(make_gripper("gripper_open", "open"))
      .build();
}

} // namespace

std::optional<step_executor::TaskPlan> TaskOrchestrator::plan(TaskId request) const {
  switch (request) {
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

} // namespace task_orchestrator
