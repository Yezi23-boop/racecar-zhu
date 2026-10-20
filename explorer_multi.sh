#!/bin/bash
# explorer_multi.sh - racecar Cartographer + 导航 一键启动
# 支持单点导航和多点循环导航
#
# 使用方法:
#   1. bash explorer_multi.sh                    # 正常启动（含 rviz2）
#   2. bash explorer_multi.sh --no-rviz          # 不启动 rviz2（用远程 rviz2 看）
#   3. bash explorer_multi.sh --save-map         # 退出时自动保存地图
#   4. bash explorer_multi.sh --no-loop          # 多点模式走完不循环
#
# 终端快捷键:
#   r: [多点模式] 开始执行导航
#   c: 清除所有点 / 停止导航
#   f: 保存地图到 ~/workspace/racecar/map/
#
# 单点模式: RViz2 Nav2 Goal 选好方向后直接执行
# 多点模式: RViz2 Nav2 Goal 选点(终端打印坐标)，按 r 执行

Run_explorer=""
SAVE_MAP=false
# 多机模式默认由远端 RViz 显示，车端不自动打开窗口。
NO_RVIZ=true
NO_LOOP=false

# 解析参数
for arg in "$@"; do
    if [ "$arg" = "--save-map" ]; then
        SAVE_MAP=true
    elif [ "$arg" = "--no-rviz" ]; then
        NO_RVIZ=true
    elif [ "$arg" = "--rviz" ]; then
        NO_RVIZ=false
    elif [ "$arg" = "--no-loop" ]; then
        NO_LOOP=true
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
        wait "$Run_explorer" 2>/dev/null
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
echo "  racecar Cartographer 导航模式"
echo "========================================"
echo "  快捷键:  r=开始 c=清除 f=保存地图"
echo "  RViz2: $([ "$NO_RVIZ" = false ] && echo "启用" || echo "禁用")"
echo "========================================"
echo ""

# 启动 Cartographer 探索模式
if [ "$NO_RVIZ" = true ]; then
    ros2 launch racecar Run_explorer.launch.py no_rviz:=true &
else
    ros2 launch racecar Run_explorer.launch.py &
fi
Run_explorer=$!

# 等待 Nav2 就绪后启动导航节点（前台运行，以获取键盘输入）
echo "等待 Nav2 就绪..."
sleep 8
if [ "$NO_LOOP" = true ]; then
    echo "启动导航节点（单次模式）..."
    ros2 run nav2_waypoint_cycle nav2_waypoint_cycle --ros-args -p enable_loop:=false
else
    echo "启动导航节点..."
    ros2 run nav2_waypoint_cycle nav2_waypoint_cycle --ros-args -p enable_loop:=true
fi
