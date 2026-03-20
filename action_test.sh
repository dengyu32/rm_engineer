#!/usr/bin/env bash
set -euo pipefail

ACTION_NAME="/move_arm"
OPTION_ID=2

usage() {
  cat << 'USAGE'
Usage:
  action_test.sh [j1 j2 j3 j4 j5 j6] [--action /move_arm] [--option 2]

Defaults:
  action   = /move_arm
  option   = 2 (joint mode)
  joints   = 0 0 0 0 0 0
USAGE
}

# Defaults
J1=0.0; J2=0.0; J3=0.0; J4=0.0; J5=0.0; J6=0.0

# Parse positional joints (first 6 numeric args)
args=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    -h|--help)
      usage
      exit 0
      ;;
    --action)
      ACTION_NAME="$2"
      shift 2
      ;;
    --option)
      OPTION_ID="$2"
      shift 2
      ;;
    *)
      args+=("$1")
      shift
      ;;
  esac
done

if [[ ${#args[@]} -gt 0 ]]; then
  if [[ ${#args[@]} -ne 6 ]]; then
    echo "Error: provide exactly 6 joint values or none." >&2
    usage
    exit 1
  fi
  J1="${args[0]}"; J2="${args[1]}"; J3="${args[2]}"; J4="${args[3]}"; J5="${args[4]}"; J6="${args[5]}"
fi

# Source environments (ROS setup uses unset vars; temporarily relax nounset)
set +u
source /opt/ros/humble/setup.bash
source /home/zc/rm_engineer/install/setup.bash
set -u

# Send goal
ros2 action send_goal "${ACTION_NAME}" engineer_interfaces/action/Move \
"{target_pose: {x: -0.35881, y: 0.24972, z: 0.48424, qx: -0.69642, qy: 0.71764, qz: 0.71764, qw: 3.4781e-05},
  target_joints: [${J1}, ${J2}, ${J3}, ${J4}, ${J5}, ${J6}],
  target_vector: {x: 0.5, y: 0.5, z: 0.0},
  option_id: ${OPTION_ID}}"
