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
if ! ros2 pkg prefix hfsm_tools >/dev/null 2>&1; then
  echo "Package 'hfsm_tools' not found. Build the workspace first:" >&2
  echo "  colcon build --symlink-install" >&2
  echo "  source install/setup.bash" >&2
  exit 1
fi
if ! ros2 bag record -h 2>&1 | grep -q "mcap"; then
  echo "rosbag2 MCAP storage plugin not available." >&2
  echo "Install with: sudo apt install ros-humble-rosbag2-storage-mcap" >&2
  exit 1
fi

COUNT="${1:-50}"

BAG_DIR="bags/move_arm_50_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$(dirname "$BAG_DIR")"

TOPICS=(
  /joint_states
  /tf
  /tf_static
  /hfsm_intents
  /test_round
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

ros2 bag record -s mcap -o "$BAG_DIR" "${TOPICS[@]}" &
BAG_PID=$!

ros2 run hfsm_tools hfsm_intent_runner --count "$COUNT"

kill -INT "$BAG_PID"
wait "$BAG_PID" || true
BAG_PID=""

mcap_hint="$BAG_DIR"/*.mcap

printf "\nRun complete.\n"
printf "Rounds executed: %s\n" "$COUNT"
printf "Bag saved to: %s\n" "$BAG_DIR"
printf "Open in Foxglove: %s\n" "$mcap_hint"
