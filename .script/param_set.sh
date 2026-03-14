# 3. 获取外部输入的参数（例如：第一个参数为 target_value）
# 如果启动时没给参数，则默认值为 1
TARGET_VALUE=${1:-1}

echo "Starting node with parameter: $TARGET_VALUE"

ros2 param set /fake_system_node fake_intent_id $TARGET_VALUE

# eg.运行脚本时输入
# param_set.sh 2