#include "task_orchestrator/static_task_orchestrator.hpp"

namespace task_orchestrator {
using namespace task_step_library;

namespace {

inline engineer_interfaces::msg::Target make_target(double x, double y, double z,
                                                     double qx, double qy,
                                                     double qz, double qw) {
  engineer_interfaces::msg::Target t;
  t.x = x;
  t.y = y;
  t.z = z;
  t.qx = qx;
  t.qy = qy;
  t.qz = qz;
  t.qw = qw;
  return t;
}

inline Step make_gripper_step(const char *label, GripperCommand cmd) {
  Step step;
  step.type = StepType::Gripper;
  step.label = label;
  step.gripper.command = cmd;
  return step;
}

inline Step make_delay_step(const char *label, int delay_ms) {
  Step step;
  step.type = StepType::Delay;
  step.label = label;
  step.delay_ms = delay_ms;
  return step;
}

inline Step make_arm_pose_step(const char *label, PlanOption option,
                               engineer_interfaces::msg::Target target,
                               int timeout_ms = 8000, int retries = 1) {
  Step step;
  step.type = StepType::ArmMove;
  step.label = label;
  step.timeout_ms = timeout_ms;
  step.max_retries = retries;
  step.arm_move.plan_option = option;
  step.arm_move.pose = target;
  return step;
}

inline Step make_arm_joint_step(const char *label,
                                const std::array<float, 6> &joints,
                                int timeout_ms = 8000, int retries = 1) {
  Step step;
  step.type = StepType::ArmMove;
  step.label = label;
  step.timeout_ms = timeout_ms;
  step.max_retries = retries;
  step.arm_move.plan_option = PlanOption::JOINTS;
  step.arm_move.joints = joints;
  return step;
}

TaskPlan make_auto_init() {
  TaskPlan plan;
  plan.task_id = TaskId::AUTO_INIT;
  plan.steps.push_back(make_gripper_step("gripper_open", GripperCommand::OPEN));
  plan.steps.push_back(make_arm_joint_step(
      "move_home_joints", std::array<float, 6>{{0.f, -1.042f, -2.618f, 0.f, 0.f, 0.f}}));
  return plan;
}

TaskPlan make_test_solve() {
  TaskPlan plan;
  plan.task_id = TaskId::TEST_SOLVE;
  plan.steps.push_back(make_arm_pose_step("move_normal_pose0", PlanOption::NORMAL,
                                          make_target(0.26235, -0.34444, 0.40285,
                                                      0.64923, 0.17886, 0.18983,
                                                      0.71447)));
  plan.steps.push_back(make_gripper_step("gripper_close", GripperCommand::CLOSE));
  plan.steps.push_back(make_delay_step("gripper_settle", 600));
  plan.steps.push_back(make_arm_pose_step("move_cartesian_pose1", PlanOption::CARTESIAN,
                                          make_target(0.26484, -0.36076, 0.55316,
                                                      0.64923, 0.17886, 0.18983,
                                                      0.71447)));
  plan.steps.push_back(make_arm_joint_step(
      "move_home_joints", std::array<float, 6>{{0.f, -1.042f, -2.618f, 0.f, 0.f, 0.f}}));
  return plan;
}

TaskPlan make_test_cartesian() {
  TaskPlan plan;
  plan.task_id = TaskId::TEST_CARTESIAN;
  plan.steps.push_back(make_arm_pose_step("approach_normal", PlanOption::NORMAL,
                                          make_target(0.44595, -0.086883, 0.61032,
                                                      -0.17604, 0.69171, -0.39348,
                                                      0.57941)));
  plan.steps.push_back(make_gripper_step("gripper_close", GripperCommand::CLOSE));
  plan.steps.push_back(make_delay_step("gripper_settle", 200));
  plan.steps.push_back(make_arm_pose_step("take_out_cartesian", PlanOption::CARTESIAN,
                                          make_target(0.54028, 0.19212, 0.27802,
                                                      -0.17604, 0.69171, -0.39348,
                                                      0.57941)));
  plan.steps.push_back(make_gripper_step("gripper_open", GripperCommand::OPEN));
  return plan;
}

TaskPlan make_auto_grab() {
  TaskPlan plan;
  plan.task_id = TaskId::AUTO_GRAB;
  plan.steps.push_back(make_arm_pose_step("auto_grab_approach_normal", PlanOption::NORMAL,
                                          make_target(0.26235, -0.34444, 0.40285,
                                                      0.64923, 0.17886, 0.18983,
                                                      0.71447)));
  plan.steps.push_back(make_gripper_step("auto_grab_gripper_close", GripperCommand::CLOSE));
  plan.steps.push_back(make_delay_step("auto_grab_gripper_settle", 600));
  plan.steps.push_back(make_arm_pose_step("auto_grab_lift_cartesian", PlanOption::CARTESIAN,
                                          make_target(0.26484, -0.36076, 0.55316,
                                                      0.64923, 0.17886, 0.18983,
                                                      0.71447)));
  plan.steps.push_back(make_arm_joint_step(
      "auto_grab_move_home", std::array<float, 6>{{0.f, -1.042f, -2.618f, 0.f, 0.f, 0.f}}));
  return plan;
}

TaskPlan make_auto_store() {
  TaskPlan plan;
  plan.task_id = TaskId::AUTO_STORE;
  plan.steps.push_back(make_arm_pose_step("auto_store_move_to_place", PlanOption::NORMAL,
                                          make_target(0.44595, -0.086883, 0.61032,
                                                      -0.17604, 0.69171, -0.39348,
                                                      0.57941)));
  plan.steps.push_back(make_delay_step("auto_store_pose_settle", 200));
  plan.steps.push_back(make_gripper_step("auto_store_gripper_open", GripperCommand::OPEN));
  plan.steps.push_back(make_delay_step("auto_store_release_settle", 300));
  plan.steps.push_back(make_arm_joint_step(
      "auto_store_move_home", std::array<float, 6>{{0.f, -1.042f, -2.618f, 0.f, 0.f, 0.f}}));
  return plan;
}

TaskPlan make_auto_get() {
  TaskPlan plan;
  plan.task_id = TaskId::AUTO_GET;
  plan.steps.push_back(make_arm_pose_step("auto_get_approach_normal", PlanOption::NORMAL,
                                          make_target(0.44595, -0.086883, 0.61032,
                                                      -0.17604, 0.69171, -0.39348,
                                                      0.57941)));
  plan.steps.push_back(make_gripper_step("auto_get_gripper_close", GripperCommand::CLOSE));
  plan.steps.push_back(make_delay_step("auto_get_gripper_settle", 200));
  plan.steps.push_back(make_arm_pose_step("auto_get_retract_cartesian", PlanOption::CARTESIAN,
                                          make_target(0.54028, 0.19212, 0.27802,
                                                      -0.17604, 0.69171, -0.39348,
                                                      0.57941)));
  plan.steps.push_back(make_arm_joint_step(
      "auto_get_move_home", std::array<float, 6>{{0.f, -1.042f, -2.618f, 0.f, 0.f, 0.f}}));
  return plan;
}

} // namespace

std::optional<TaskPlan>
StaticTaskOrchestrator::plan(const TaskRequest &request) const {
  switch (request.task_id) {
  case TaskId::AUTO_INIT:
    return make_auto_init();
  case TaskId::TEST_SOLVE:
    return make_test_solve();
  case TaskId::TEST_CARTESIAN:
    return make_test_cartesian();
  case TaskId::AUTO_GRAB:
    return make_auto_grab();
  case TaskId::AUTO_STORE:
    return make_auto_store();
  case TaskId::AUTO_GET:
    return make_auto_get();
  case TaskId::IDLE:
  default:
    return std::nullopt;
  }
}

} // namespace task_orchestrator
