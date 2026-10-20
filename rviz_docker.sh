#!/bin/bash
# 本地 Docker RViz2 可视化远程无人车
# 用法: ./rviz_docker.sh
#
# 前提: docker pull ros:humble (或 osrf/ros:humble-desktop)
# 原理: Docker --network host + ROS_DOMAIN_ID=0 直接发现同网段的 ROS2 话题

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
RVIZ_CONFIG="$SCRIPT_DIR/rviz_remote.rviz"
IMAGE=""

# 检测可用的 Docker 镜像
if docker image inspect osrf/ros:humble-desktop &>/dev/null; then
    IMAGE="osrf/ros:humble-desktop"
    echo "✅ 使用镜像: $IMAGE (已含 rviz2)"
    NEED_INSTALL=false
elif docker image inspect ros:humble &>/dev/null; then
    IMAGE="ros:humble"
    echo "⚠️  使用镜像: $IMAGE (需要安装 rviz2)"
    NEED_INSTALL=true
else
    echo "❌ 没有找到 Humble Docker 镜像!"
    echo "请先运行: docker pull ros:humble"
    echo "或者:     docker pull osrf/ros:humble-desktop"
    exit 1
fi

# 允许 Docker 访问 X11
xhost +local:docker 2>/dev/null || true

CONTAINER_NAME="rviz2_viewer"

# 清除旧容器
docker rm -f $CONTAINER_NAME 2>/dev/null || true

echo ""
echo "🚀 启动 RViz2 可视化..."
echo "   远程车 DDS 发现: --network host + ROS_DOMAIN_ID=0"
echo "   RViz 配置: $RVIZ_CONFIG"
echo ""

if [ "$NEED_INSTALL" = true ]; then
    echo "📦 首次运行需要安装 rviz2，大约需要 2-3 分钟..."
    docker run -it --rm \
        --name $CONTAINER_NAME \
        --network host \
        --privileged \
        -e DISPLAY=$DISPLAY \
        -e QT_X11_NO_MITSHM=1 \
        -e ROS_DOMAIN_ID=0 \
        -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
        -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
        -v "$RVIZ_CONFIG":/opt/rviz_config.rviz:ro \
        $IMAGE \
        bash -c '
            echo "正在安装 rviz2..."
            apt-get update -qq && apt-get install -y -qq ros-humble-rviz2 ros-humble-nav2-rviz-plugins 2>&1 | tail -5
            echo "✅ 安装完成，启动 RViz2..."
            source /opt/ros/humble/setup.bash
            export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
            export ROS_DOMAIN_ID=0
            rviz2 -d /opt/rviz_config.rviz
        '
else
    docker run -it --rm \
        --name $CONTAINER_NAME \
        --network host \
        --privileged \
        -e DISPLAY=$DISPLAY \
        -e QT_X11_NO_MITSHM=1 \
        -e ROS_DOMAIN_ID=0 \
        -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
        -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
        -v "$RVIZ_CONFIG":/opt/rviz_config.rviz:ro \
        $IMAGE \
        bash -c '
            source /opt/ros/humble/setup.bash
            export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
            export ROS_DOMAIN_ID=0
            rviz2 -d /opt/rviz_config.rviz
        '
fi
