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


# 启动 ROS 2 launch 文件

echo "启动 ROS 2 车辆与 RF2O: Run_car.launch.py（不启动旧 EKF）"
ros2 launch racecar Run_car.launch.py &
Run_car=$!

# 等待5秒
sleep 10


# 启动 nav2 节点
 echo "启动 nav2 节点"
 ros2 launch racecar Run_nav.launch.py &
 Run_nav=$!

echo "所有节点已启动"

#Wait for all background processes to complete
wait "$Run_car" "$Run_nav"
