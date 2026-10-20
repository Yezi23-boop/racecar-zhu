#!/bin/bash

Run_car=""
Run_nav=""

# Function to handle termination
terminate() {
  echo "终止所有后台进程..."
  if [ -n "$Run_nav" ] && kill -0 "$Run_nav" 2>/dev/null; then
    kill "$Run_nav"
    wait "$Run_nav"
  fi
  if [ -n "$Run_car" ] && kill -0 "$Run_car" 2>/dev/null; then
    kill "$Run_car"
    wait "$Run_car"
  fi
  echo "所有进程已终止。"
  exit 0
}

# Trap Ctrl+C (SIGINT)
trap terminate SIGINT SIGTERM

WORKSPACE_DIR=$(cd "$(dirname "$0")" && pwd)
cd "$WORKSPACE_DIR" || exit 1
source /opt/ros/humble/setup.bash
source "$WORKSPACE_DIR/install/setup.bash"

# 启动 ROS 2 launch 文件

echo "启动单点导航底层：Cartographer 激光+IMU里程计（不启动 RF2O）"
ros2 launch racecar Run_car_one.launch.py use_rviz:=false &
Run_car=$!

# 等待5秒
sleep 5



 echo "启动 Nav2：NavFn + Regulated Pure Pursuit（Ackermann 无旋转恢复）"
 ros2 launch racecar Run_nav.launch.py  &
 Run_nav=$!

echo "所有节点已启动"

#Wait for all background processes to complete
wait "$Run_car" "$Run_nav"
