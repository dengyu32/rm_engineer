#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$(cd "${SCRIPT_DIR}/../.." && pwd)"
cd "$WORKSPACE"

# 可通过环境变量覆盖的默认配置。
ACTION_NAME="${ACTION_NAME:-/move_arm}"
ACTION_TYPE="${ACTION_TYPE:-engineer_interfaces/action/Move}"
GOAL_FILE_DEFAULT="${SCRIPT_DIR}/run_move_arm_multi_goals.txt"

# 运行时参数默认值。
COUNT=1
GOAL_FILE="$GOAL_FILE_DEFAULT"
SLEEP_SEC="${SLEEP_SEC:-5.0}"
OPTION_ID="${OPTION_ID:-0}"
ENABLE_BAG="${ENABLE_BAG:-0}"
BAG_DIR="bags/move_arm_multi_$(date +%Y%m%d_%H%M%S)"
TOPICS=(
  /joint_states
  /tf
  /tf_static
  /move_arm/_action/status
  /move_arm/_action/feedback
)

usage() {
  cat <<EOF
用法:
  $(basename "$0") [次数] [目标文件]

示例:
  $(basename "$0")
  $(basename "$0") 3
  $(basename "$0") 1 .script/test/run_move_arm_multi_goals.txt

目标文件格式:
  每个目标只写两行, 空行分隔:

    position 0.56392 0.17588 0.65313
    orientation 0.5552 0.50536 -0.50233 -0.42898

环境变量:
  ACTION_NAME   默认: ${ACTION_NAME}
  OPTION_ID     默认: ${OPTION_ID}   (0=普通方式, 1=笛卡尔, 2=关节)
  SLEEP_SEC     默认: ${SLEEP_SEC}
  ENABLE_BAG    默认: ${ENABLE_BAG}   (1=录包)
EOF
}

is_number() {
  [[ "$1" =~ ^[-+]?[0-9]*\.?[0-9]+([eE][-+]?[0-9]+)?$ ]]
}

trim() {
  local value="$1"
  value="${value#"${value%%[![:space:]]*}"}"
  value="${value%"${value##*[![:space:]]}"}"
  printf '%s' "$value"
}

parse_args() {
  local args=()
  while [[ $# -gt 0 ]]; do
    case "$1" in
      -h|--help)
        usage
        exit 0
        ;;
      *)
        args+=("$1")
        shift
        ;;
    esac
  done

  if [[ ${#args[@]} -ge 1 ]]; then
    # 第一个参数优先解释为循环次数；如果不是数字，则视为目标文件路径。
    if is_number "${args[0]}"; then
      COUNT="${args[0]}"
    else
      GOAL_FILE="${args[0]}"
    fi
  fi

  if [[ ${#args[@]} -ge 2 ]]; then
    GOAL_FILE="${args[1]}"
  fi

  if [[ ${#args[@]} -gt 2 ]]; then
    echo "参数过多。" >&2
    usage
    exit 1
  fi
}

validate_env() {
  # 尽量自动加载 ROS 与工作区环境，降低手动 source 的负担。
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
    echo "未找到 ros2，请先 source ROS 2 环境。" >&2
    exit 1
  fi

  if [[ ! -f "$GOAL_FILE" ]]; then
    echo "目标文件不存在: $GOAL_FILE" >&2
    exit 1
  fi

  if ! [[ "$COUNT" =~ ^[1-9][0-9]*$ ]]; then
    echo "次数必须是正整数，当前值: $COUNT" >&2
    exit 1
  fi

  if ! [[ "$OPTION_ID" =~ ^[0-2]$ ]]; then
    echo "OPTION_ID 只能是 0、1 或 2，当前值: $OPTION_ID" >&2
    exit 1
  fi
}

build_goals() {
  GOALS=()
  local line
  local line_no=0
  local current_position=""
  local current_orientation=""

  # 读取“position + orientation”文本，拼成 ros2 action 所需的完整 goal YAML。
  while IFS= read -r line || [ -n "$line" ]; do
    line_no=$((line_no + 1))
    line="$(trim "$line")"

    # 允许空行和注释，方便手工维护目标文件。
    if [[ -z "$line" || "${line:0:1}" == "#" ]]; then
      continue
    fi

    read -r -a parts <<< "$line"
    local key="${parts[0]}"

    case "$key" in
      position)
        if [[ -n "$current_position" && -z "$current_orientation" ]]; then
          echo "第 ${line_no} 行之前已有未完成的 position，请先补一行 orientation。" >&2
          exit 1
        fi
        if [[ ${#parts[@]} -ne 4 ]]; then
          echo "第 ${line_no} 行格式错误: position 需要 3 个数值。" >&2
          exit 1
        fi
        for value in "${parts[@]:1}"; do
          if ! is_number "$value"; then
            echo "第 ${line_no} 行包含非法数值: $value" >&2
            exit 1
          fi
        done
        current_position="${parts[1]} ${parts[2]} ${parts[3]}"
        ;;
      orientation)
        if [[ -z "$current_position" ]]; then
          echo "第 ${line_no} 行出现了 orientation，但前面没有对应的 position。" >&2
          exit 1
        fi
        if [[ -n "$current_orientation" ]]; then
          echo "第 ${line_no} 行之前已有未完成的 orientation，请检查目标文件格式。" >&2
          exit 1
        fi
        if [[ ${#parts[@]} -ne 5 ]]; then
          echo "第 ${line_no} 行格式错误: orientation 需要 4 个数值。" >&2
          exit 1
        fi
        for value in "${parts[@]:1}"; do
          if ! is_number "$value"; then
            echo "第 ${line_no} 行包含非法数值: $value" >&2
            exit 1
          fi
        done
        current_orientation="${parts[1]} ${parts[2]} ${parts[3]} ${parts[4]}"
        ;;
      *)
        echo "第 ${line_no} 行无法识别: $key" >&2
        echo "只支持 position / orientation。" >&2
        exit 1
        ;;
    esac

    if [[ -n "$current_position" && -n "$current_orientation" ]]; then
      read -r px py pz <<< "$current_position"
      read -r qx qy qz qw <<< "$current_orientation"
      # 这里只走 pose 模式，因此 joints/vector/length 统一补零。
      GOALS+=("{target_pose: {x: ${px}, y: ${py}, z: ${pz}, qx: ${qx}, qy: ${qy}, qz: ${qz}, qw: ${qw}}, target_joints: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], target_vector: {x: 0.0, y: 0.0, z: 0.0}, target_length: 0.0, option_id: ${OPTION_ID}}")
      current_position=""
      current_orientation=""
    fi
  done < "$GOAL_FILE"

  if [[ -n "$current_position" || -n "$current_orientation" ]]; then
    echo "目标文件末尾存在未配对的 position / orientation。" >&2
    exit 1
  fi

  if [[ ${#GOALS[@]} -eq 0 ]]; then
    echo "目标文件中没有可发送的目标: $GOAL_FILE" >&2
    exit 1
  fi
}

cleanup() {
  if [ -n "${BAG_PID:-}" ] && kill -0 "$BAG_PID" 2>/dev/null; then
    kill -INT "$BAG_PID" || true
    wait "$BAG_PID" || true
  fi
}
trap cleanup INT TERM EXIT

parse_args "$@"
validate_env
build_goals

# 可选录包，用于回放或排查动作执行问题。
if [ "$ENABLE_BAG" = "1" ]; then
  mkdir -p "$(dirname "$BAG_DIR")"
  ros2 bag record -o "$BAG_DIR" "${TOPICS[@]}" &
  BAG_PID=$!
fi

printf "动作名: %s\n" "$ACTION_NAME"
printf "目标文件: %s\n" "$GOAL_FILE"
printf "目标数量: %s\n" "${#GOALS[@]}"
printf "循环次数: %s\n" "$COUNT"
printf "模式(option_id): %s\n" "$OPTION_ID"
printf "间隔秒数: %s\n\n" "$SLEEP_SEC"

# 外层控制整组目标重复次数，内层顺序发送每一个目标。
for ((cycle = 1; cycle <= COUNT; cycle++)); do
  printf "第 %s/%s 轮\n" "$cycle" "$COUNT"
  for ((index = 0; index < ${#GOALS[@]}; index++)); do
    printf "  发送目标 %s/%s\n" "$((index + 1))" "${#GOALS[@]}"
    ros2 action send_goal "$ACTION_NAME" "$ACTION_TYPE" "${GOALS[index]}"
    sleep "$SLEEP_SEC"
  done
done

printf "\n全部目标发送完成。\n"

if [ "$ENABLE_BAG" = "1" ]; then
  kill -INT "$BAG_PID"
  wait "$BAG_PID" || true
  BAG_PID=""
  printf "Bag 保存路径: %s\n" "$BAG_DIR"
fi
