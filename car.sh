#!/bin/bash
# Function to handle termination
Run_car=""

terminate() {
  echo "终止所有后台进程..."
  if [ -n "$Run_car" ] && kill -0 "$Run_car" 2>/dev/null; then
    kill "$Run_car"
    wait "$Run_car"
  fi
  echo "所有进程已终止。"
  exit 0
}

# Trap Ctrl+C (SIGINT)
trap terminate SIGINT SIGTERM

# 设置 ROS 2 环境变量
WORKSPACE_DIR=$(cd "$(dirname "$0")" && pwd)

# 优先 source Humble 环境 (针对 IP 192.168.3.8)
if [ -f "/opt/ros/humble/setup.bash" ]; then
  source /opt/ros/humble/setup.bash
else
  source /opt/ros/foxy/setup.bash
fi

if [ -f "$WORKSPACE_DIR/install/setup.bash" ]; then
  source "$WORKSPACE_DIR/install/setup.bash"
fi

# 启动底盘、激光雷达和IMU，但不启用EKF
ros2 launch racecar Run_car.launch.py  &
Run_car=$!
wait "$Run_car"
