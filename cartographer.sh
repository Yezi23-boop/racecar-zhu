#!/bin/bash

Run_cartographer=""
Run_cartographer_pgid=""
Run_rviz=""
Run_rviz_pgid=""
USE_RVIZ=false
terminating=0

for arg in "$@"; do
  case "$arg" in
    --rviz)
      USE_RVIZ=true
      ;;
    --no-rviz)
      USE_RVIZ=false
      ;;
  esac
done

terminate() {
  if [ "$terminating" -eq 1 ]; then
    return
  fi
  terminating=1

  echo "终止 Cartographer 建图和 RViz2 进程..."

  for pgid in "$Run_cartographer_pgid" "$Run_rviz_pgid"; do
    if [ -n "$pgid" ]; then
      kill -TERM -- "-$pgid" 2>/dev/null || true
    fi
  done

  for pid in "$Run_cartographer" "$Run_rviz"; do
    if [ -n "$pid" ]; then
      wait "$pid" 2>/dev/null || true
    fi
  done

  echo "所有进程已终止。"
}

WORKSPACE_DIR=$(cd "$(dirname "$0")" && pwd) || exit 1
cd "$WORKSPACE_DIR" || exit 1

if [ ! -f /opt/ros/humble/setup.bash ]; then
  echo "错误：未找到 /opt/ros/humble/setup.bash"
  exit 1
fi
source /opt/ros/humble/setup.bash

if [ ! -f "$WORKSPACE_DIR/install/setup.bash" ]; then
  echo "错误：未找到 $WORKSPACE_DIR/install/setup.bash"
  echo "请先在工作空间执行 colcon build。"
  exit 1
fi
source "$WORKSPACE_DIR/install/setup.bash"

RVIZ_CONFIG="$WORKSPACE_DIR/src/racecar/rviz/racecar_cartographer.rviz"
if [ "$USE_RVIZ" = true ] && [ ! -f "$RVIZ_CONFIG" ]; then
  echo "错误：未找到 RViz2 配置文件：$RVIZ_CONFIG"
  exit 1
fi

trap terminate SIGINT SIGTERM EXIT

echo "启动 Cartographer 纯建图模式"
echo "包含：底盘、雷达、IMU、静态 TF、Cartographer"
echo "不包含：RF2O、EKF、AMCL、Nav2"

setsid ros2 launch racecar Run_cartographer_mapping.launch.py &
Run_cartographer=$!
Run_cartographer_pgid=$Run_cartographer

if [ "$USE_RVIZ" = true ]; then
  echo "启动 RViz2：$RVIZ_CONFIG"
  if [ -z "${DISPLAY:-}" ] && [ -z "${WAYLAND_DISPLAY:-}" ]; then
    echo "警告：当前终端没有 DISPLAY 或 WAYLAND_DISPLAY，RViz2 弹窗可能无法显示。"
  fi
  setsid rviz2 -d "$RVIZ_CONFIG" &
  Run_rviz=$!
  Run_rviz_pgid=$Run_rviz
else
  echo "RViz2 已禁用（如需开启：bash cartographer.sh --rviz）"
fi

echo "Cartographer 建图已启动。"
echo "保存地图请另开终端执行："
echo "ros2 run nav2_map_server map_saver_cli -f $WORKSPACE_DIR/src/racecar/map/carto_map"

wait "$Run_cartographer"
