#!/bin/bash
# explorer.sh - racecar Cartographer 自主探索模式一键启动
# 架构: 底盘+雷达+IMU → Cartographer SLAM → Nav2 导航
#
# 用法:
#   bash explorer.sh              # 正常启动（含 rviz2）
#   bash explorer.sh --save-map   # 退出时自动保存地图
#   bash explorer.sh --no-rviz    # 不启动 rviz2
#   bash explorer.sh --save-map --no-rviz  # 保存地图 + 不启动 rviz2

Run_explorer=""
SAVE_MAP=false
NO_RVIZ=false

# 解析参数
for arg in "$@"; do
    if [ "$arg" = "--save-map" ]; then
        SAVE_MAP=true
    elif [ "$arg" = "--no-rviz" ]; then
        NO_RVIZ=true
    fi
done

terminate() {
    echo ""
    echo "========================================"
    echo "  终止 Cartographer 自主探索..."
    echo "========================================"

    if [ "$SAVE_MAP" = true ]; then
        echo "正在保存地图..."
        WORKSPACE_DIR=$(cd "$(dirname "$0")" && pwd)
        MAP_DIR="$WORKSPACE_DIR/src/racecar/map"
        TIMESTAMP=$(date +%Y%m%d_%H%M%S)
        MAP_NAME="carto_map_${TIMESTAMP}"

        # 通过 Cartographer 序列化保存 pbstream
        ros2 service call /finish_trajectory cartographer_ros_msgs/srv/FinishTrajectory "{trajectory_id: 0}" 2>/dev/null
        sleep 1
        ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "{filename: '${MAP_DIR}/${MAP_NAME}.pbstream'}" 2>/dev/null
        sleep 1

        # 同时通过 map_saver 保存为 pgm+yaml（Nav2 可用格式）
        ros2 run nav2_map_server map_saver_cli -f "${MAP_DIR}/${MAP_NAME}" --ros-args -p save_map_timeout:=5.0 2>/dev/null
        echo "地图已保存到: ${MAP_DIR}/${MAP_NAME}"
    fi

    if [ -n "$Run_explorer" ] && kill -0 "$Run_explorer" 2>/dev/null; then
        kill "$Run_explorer"
        wait "$Run_explorer"
    fi
    echo "所有进程已终止。"
    exit 0
}

trap terminate SIGINT SIGTERM

# 设置 ROS 2 环境变量
WORKSPACE_DIR=$(cd "$(dirname "$0")" && pwd)

if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
else
    source /opt/ros/foxy/setup.bash
fi

if [ -f "$WORKSPACE_DIR/install/setup.bash" ]; then
    source "$WORKSPACE_DIR/install/setup.bash"
fi

# 注意：不要在远程车上设置 FASTRTPS_DEFAULT_PROFILES_FILE
# 使用默认 DDS 组播发现，否则本机节点间通信会异常
# 本机 RViz2 通过自己的 FastDDS peer 配置来发现远程话题

echo "========================================"
echo "  racecar Cartographer 自主探索模式"
echo "========================================"
echo "  TF链路: map→odom→base_footprint→laser_link/IMU_link"
echo "  定位: Cartographer SLAM (激光+IMU, 全由Cartographer提供)"
echo "  导航: Nav2 (navigation_launch, 无 AMCL)"
echo "  RViz2: $([ "$NO_RVIZ" = false ] && echo "启用" || echo "禁用")"
echo "  保存地图: $([ "$SAVE_MAP" = true ] && echo "退出时自动保存" || echo "Ctrl+C 退出（不保存）")"
echo "========================================"
echo ""

# 启动 Cartographer 探索模式
if [ "$NO_RVIZ" = true ]; then
    ros2 launch racecar Run_explorer.launch.py no_rviz:=true &
else
    ros2 launch racecar Run_explorer.launch.py &
fi
Run_explorer=$!
wait "$Run_explorer"
