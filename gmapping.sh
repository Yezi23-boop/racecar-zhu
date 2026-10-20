#!/bin/bash

Run_car_pid=""
Run_car_pgid=""
gmapping_pid=""
gmapping_pgid=""
terminating=0
use_rviz=false

terminate() {
  if [ "$terminating" -eq 1 ]; then
    return
  fi
  terminating=1

  echo "终止 GMapping 建图相关进程..."

  for pgid in "$gmapping_pgid" "$Run_car_pgid"; do
    if [ -n "$pgid" ]; then
      kill -TERM -- "-$pgid" 2>/dev/null || true
    fi
  done

  for pid in "$gmapping_pid" "$Run_car_pid"; do
    if [ -n "$pid" ]; then
      wait "$pid" 2>/dev/null || true
    fi
  done

  echo "GMapping 建图相关进程已终止。"
}

trap terminate SIGINT SIGTERM EXIT

case "${1:-}" in
  --rviz)
    use_rviz=true
    ;;
  "")
    ;;
  *)
    echo "用法: bash gmapping.sh [--rviz]"
    exit 2
    ;;
esac

cd "$(dirname "$0")" || exit 1

if [ -f /opt/ros/humble/setup.bash ]; then
  source /opt/ros/humble/setup.bash
fi

if [ -f install/setup.bash ]; then
  source install/setup.bash
fi

echo "启动底盘、激光雷达、IMU 和 RF2O（RViz: $use_rviz）"
setsid ros2 launch racecar Run_car.launch.py use_rviz:="$use_rviz" &
Run_car_pid=$!
Run_car_pgid=$Run_car_pid

sleep 8

echo "启动 GMapping：map -> odom；RF2O：odom -> base_footprint"
setsid ros2 launch slam_gmapping slam_gmapping.launch.py \
  scan_topic:=/scan \
  base_frame:=base_footprint \
  odom_frame:=odom \
  map_frame:=map &
gmapping_pid=$!
gmapping_pgid=$gmapping_pid

echo "GMapping 建图已启动。"
echo "显示 RViz: bash gmapping.sh --rviz"
echo "保存地图: ros2 run nav2_map_server map_saver_cli -f src/racecar/map/gmapping_map"

wait "$Run_car_pid" "$gmapping_pid"
