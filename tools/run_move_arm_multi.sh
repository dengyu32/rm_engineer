#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "$WORKSPACE"

set +u
if [ -f /opt/ros/humble/setup.bash ]; then
  source /opt/ros/humble/setup.bash
fi
if [ -f install/setup.bash ]; then
  source install/setup.bash
elif [ -f .colcon/install/setup.bash ]; then
  source .colcon/install/setup.bash
fi
set -u

if ! command -v ros2 >/dev/null 2>&1; then
  echo "ros2 not found. Please source your ROS 2 environment first." >&2
  exit 1
fi

ACTION_NAME="/move_arm"
ACTION_TYPE="engineer_interfaces/action/Move"

# Number of cycles to repeat all goals
COUNT="${1:-50}"

# -------------------------------------------------------------------
# Add goals below. Each entry is a full YAML goal (single line).
# You can mix different option_id values in this list.
# -------------------------------------------------------------------
GOALS=(
  "{target_pose: {x: 0.0, y: 0.0, z: 0.0, qx: 0.0, qy: 0.0, qz: 0.0, qw: 1.0}, target_joints: [0.0, 0.7155, -2.2340, 0.0, 0.0, 1.5707], target_vector: {x: 0.0, y: 0.0, z: 0.0}, option_id: 2}"
  "{target_pose: {x: -0.32138, y: 0.24944, z: 0.4624, qx: -0.69021, qy: 0.00053466, qz: 0.72359, qw: -0.0040804}, target_joints: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], target_vector: {x: 0.0, y: 0.0, z: 0.0}, option_id: 0}"
  "{target_pose: {x: 0.0, y: 0.0, z: 0.0, qx: 0.0, qy: 0.0, qz: 0.0, qw: 1.0}, target_joints: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], target_vector: {x: -1.0, y: 0.0, z: 0.0}, option_id: 1}"
  "{target_pose: {x: 0.0, y: 0.0, z: 0.0, qx: 0.0, qy: 0.0, qz: 0.0, qw: 1.0}, target_joints: [3.1415, 0.7504, -2.2689, 0.0, 0.0, -1.5707], target_vector: {x: 0.0, y: 0.0, z: 0.0}, option_id: 2}"
  "{target_pose: {x: 0.0, y: 0.0, z: 0.0, qx: 0.0, qy: 0.0, qz: 0.0, qw: 1.0}, target_joints: [0.0, 0.7155, -2.2340, 0.0, 0.0, 1.5707], target_vector: {x: 0.0, y: 0.0, z: 0.0}, option_id: 2}"
  "{target_pose: {x: -0.32138, y: 0.24944, z: 0.4624, qx: -0.69021, qy: 0.00053466, qz: 0.72359, qw: -0.0040804}, target_joints: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], target_vector: {x: 0.0, y: 0.0, z: 0.0}, option_id: 0}"
)

# Delay between goals (seconds)
SLEEP_SEC="${SLEEP_SEC:-1.0}"

# Rosbag recording (set ENABLE_BAG=1 to record)
ENABLE_BAG="${ENABLE_BAG:-0}"
BAG_DIR="bags/move_arm_multi_$(date +%Y%m%d_%H%M%S)"
TOPICS=(
  /joint_states
  /tf
  /tf_static
  /move_arm/_action/status
  /move_arm/_action/feedback
)

cleanup() {
  if [ -n "${BAG_PID:-}" ] && kill -0 "$BAG_PID" 2>/dev/null; then
    kill -INT "$BAG_PID" || true
    wait "$BAG_PID" || true
  fi
}
trap cleanup INT TERM EXIT

if [ "$ENABLE_BAG" = "1" ]; then
  mkdir -p "$(dirname "$BAG_DIR")"
  ros2 bag record -o "$BAG_DIR" "${TOPICS[@]}" &
  BAG_PID=$!
fi
for ((i=1; i<=COUNT; i++)); do
  echo "Cycle ${i}/${COUNT}"
  for goal in "${GOALS[@]}"; do
    echo "Sending goal: $goal"
    ros2 action send_goal "$ACTION_NAME" "$ACTION_TYPE" "$goal"
    sleep "$SLEEP_SEC"
  done
done

printf "\nAll goals sent.\n"

if [ "$ENABLE_BAG" = "1" ]; then
  kill -INT "$BAG_PID"
  wait "$BAG_PID" || true
  BAG_PID=""
  printf "Bag saved to: %s\n" "$BAG_DIR"
fi
