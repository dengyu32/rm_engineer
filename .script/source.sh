#!/bin/bash

# 1. 设置工作空间的绝对路径（根据你的实际路径修改）
WORKSPACE_PATH="$HOME/rm_engineer"

# 2. Source 当前工作空间
if [ -f "$WORKSPACE_PATH/.colcon/install/setup.bash" ]; then
    source "$WORKSPACE_PATH/.colcon/install/setup.bash"
    echo "Successfully sourced workspace: $WORKSPACE_PATH"
else
    echo "Error: Workspace setup.bash not found at $WORKSPACE_PATH"
    exit 1
fi