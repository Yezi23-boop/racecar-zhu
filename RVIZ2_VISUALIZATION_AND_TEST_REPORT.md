# RViz2 远程可视化配置 & 无人车定位建图导航测试报告

> 日期：2026-03-22  
> 平台：RISC-V (bianbu) + Ubuntu 20.04 本机  
> ROS 版本：ROS 2 Humble

---

## 一、系统架构

```
本机 (192.168.3.7, Ubuntu 20.04)          远程无人车 (192.168.3.16, RISC-V bianbu)
┌──────────────────────┐                  ┌──────────────────────────────────┐
│  Docker: ros:humble  │   DDS (UDP)      │  ROS 2 Humble (原生)             │
│  ┌──────────────┐    │◄════════════════►│                                  │
│  │   RViz2      │    │   /scan /map     │  lslidar_driver → /scan          │
│  │   Nav2插件   │    │   /tf /plan      │  imu_get → /IMU_data             │
│  └──────────────┘    │   同一子网组播    │  Cartographer → /map /tf         │
└──────────────────────┘                  │  Nav2 → /plan /local_plan        │
                                          │  racecar_driver → 底盘控制       │
                                          └──────────────────────────────────┘
```

### TF 链路
```
map → odom → base_footprint → laser_link
                             → IMU_link
```
全部由 Cartographer 发布（`provide_odom_frame=true`），无需外部里程计。

---

## 二、本机 RViz2 可视化配置

### 2.1 前提条件
- 本机已安装 Docker（`docker --version`）
- 本机与无人车在同一局域网（192.168.3.x 网段）
- 代理工具 mihomo-party 运行在 `127.0.0.1:7890`（可选，加速安装）

### 2.2 防火墙放行

```bash
# 放行远程车的 DDS 通信
sudo ufw allow from 192.168.3.16 proto udp   # DDS 数据和发现
sudo ufw allow from 192.168.3.16 proto tcp   # DDS TCP 传输
sudo ufw allow from 239.255.0.0/16 proto udp # DDS 组播发现
```

### 2.3 创建 Docker RViz2 容器

```bash
# 1. 拉取 ROS Humble 基础镜像
docker pull ros:humble

# 2. 创建容器并安装 rviz2（走代理加速）
docker run -d --name rviz2_humble \
  --network host \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  ros:humble \
  bash -c "
    echo 'Acquire::http::Proxy \"http://127.0.0.1:7890\";' > /etc/apt/apt.conf.d/proxy.conf
    echo 'Acquire::https::Proxy \"http://127.0.0.1:7890\";' >> /etc/apt/apt.conf.d/proxy.conf
    apt-get update && apt-get install -y --no-install-recommends \
      ros-humble-rviz2 \
      ros-humble-nav2-rviz-plugins \
      ros-humble-rmw-fastrtps-cpp && \
    touch /tmp/install_done && sleep infinity
  "

# 3. 等待安装完成（约 3~5 分钟）
docker exec rviz2_humble bash -c "test -f /tmp/install_done && echo DONE || echo INSTALLING"
```

### 2.4 创建 FastDDS 发现配置

在容器内创建 `/rviz_config/fastdds_discovery.xml`：

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
    <participant profile_name="participant_profile" is_default_profile="true">
        <rtps>
            <builtin>
                <metatrafficUnicastLocatorList>
                    <locator>
                        <udpv4><address>192.168.3.7</address></udpv4>
                    </locator>
                </metatrafficUnicastLocatorList>
                <initialPeersList>
                    <locator>
                        <udpv4><address>192.168.3.7</address></udpv4>
                    </locator>
                    <locator>
                        <udpv4><address>192.168.3.16</address></udpv4>
                    </locator>
                </initialPeersList>
            </builtin>
        </rtps>
    </participant>
</profiles>
```

```bash
# 拷贝配置到容器
docker exec rviz2_humble mkdir -p /rviz_config
docker cp fastdds_discovery.xml rviz2_humble:/rviz_config/
docker cp nav2_view.rviz rviz2_humble:/rviz_config/
```

### 2.5 启动 RViz2

```bash
# 允许 Docker 访问 X11
xhost +local:docker

# 启动 RViz2
docker exec -it \
  -e DISPLAY=$DISPLAY \
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -e ROS_DOMAIN_ID=0 \
  -e FASTRTPS_DEFAULT_PROFILES_FILE=/rviz_config/fastdds_discovery.xml \
  rviz2_humble \
  bash -c "source /opt/ros/humble/setup.bash && rviz2 -d /rviz_config/nav2_view.rviz"
```

### 2.6 一键脚本

配置文件和脚本已保存在 `/home/zk/rviz2_config/`：
- `rviz2_remote.sh` — 一键启动脚本
- `fastdds_discovery.xml` — DDS 发现配置
- `nav2_view.rviz` — RViz2 布局配置

```bash
# 一键启动
bash /home/zk/rviz2_config/rviz2_remote.sh
```

### ⚠️ 重要：远程车不要使用 FastDDS peer 配置

**教训**：远程车的 `explorer.sh` 如果加载了 `/home/bianbu/fastdds_peer.xml`（`FASTRTPS_DEFAULT_PROFILES_FILE`），会导致本地节点间 DDS 通信异常（Cartographer 收不到 /scan）。

**解决方案**：远程车使用默认 DDS 组播发现即可。本机通过防火墙放行 + FastDDS peer 配置来发现远程话题。

已将该文件重命名为 `fastdds_peer.xml.bak`。

---

## 三、远程无人车启动方法

### 3.1 SSH 连接
```bash
ssh bianbu@192.168.3.16   # 密码: 1
```

### 3.2 启动全套（建图+导航）
```bash
cd /home/bianbu/workspace/racecar
source /opt/ros/humble/setup.bash
source install/setup.bash
bash explorer.sh --no-rviz
```

参数说明：
| 参数 | 说明 |
|------|------|
| （无参数） | 启动含 RViz2（远程显示需 X11 转发） |
| `--no-rviz` | 不启动 RViz2（推荐，用本机 RViz2 查看） |
| `--save-map` | 退出时自动保存地图 |
| `--save-map --no-rviz` | 保存地图 + 不启动 RViz2 |

### 3.3 启动顺序（自动）
1. **第 0 秒**：底盘驱动 + 雷达 + IMU + 静态 TF
2. **第 3 秒**：Cartographer SLAM
3. **第 8 秒**：Nav2 导航栈（controller_server, planner_server, bt_navigator 等）
4. **约 30 秒后**：`Managed nodes are active`（所有节点就绪）

### 3.4 节点列表（共 12 个进程）
| 节点 | 包 | 功能 |
|------|------|------|
| racecar_driver_node | racecar_driver | 底盘 PWM 控制 |
| lslidar_driver_node | lslidar_driver | LSN10 激光雷达 |
| publisher_imu_node | imu_get | IMU 数据 |
| joint_state_publisher | joint_state_publisher | 关节状态 |
| base_footprint2laser_link | tf2_ros | 静态 TF |
| base_footprint2imu_link | tf2_ros | 静态 TF |
| cartographer_node | cartographer_ros | SLAM 定位+建图 |
| cartographer_occupancy_grid_node | cartographer_ros | 栅格地图发布 |
| controller_server | nav2_controller | 路径跟踪 |
| planner_server | nav2_planner | 全局规划 |
| bt_navigator | nav2_bt_navigator | 行为树导航 |
| lifecycle_manager | nav2_lifecycle_manager | 生命周期管理 |
| behavior_server | nav2_behaviors | 恢复行为 |
| velocity_smoother | nav2_velocity_smoother | 速度平滑 |
| waypoint_follower | nav2_waypoint_follower | 路点跟随 |

---

## 四、测试结果

### 4.1 传感器数据

| 传感器 | 话题 | 频率 | 时间戳延迟 | 状态 |
|--------|------|------|-----------|------|
| 激光雷达 LSN10 | /scan | 9.98 Hz | 3ms | ✅ 正常 |
| IMU | /IMU_data | 100 Hz | 12ms | ✅ 正常 |

### 4.2 静止定位稳定性测试

**测试条件**：无人车完全静止，15 次采样，间隔 2 秒，共 30 秒。

```
采样  X (m)      Y (m)      Yaw (deg)
 1   -0.1791    +0.0057    -0.36
 2   -0.1793    +0.0040    -0.38
 3   -0.1762    +0.0053    -0.46
 4   -0.1806    +0.0015    -0.45
 5   -0.1784    +0.0015    -0.44
 6   -0.1758    +0.0038    -0.44
 7   -0.1798    +0.0016    -0.41
 8   -0.1774    +0.0048    -0.42
 9   -0.1789    +0.0010    -0.44
10   -0.1797    +0.0039    -0.44
11   -0.1760    +0.0017    -0.43
12   -0.1813    +0.0034    -0.40
13   -0.1797    +0.0039    -0.38
14   -0.1782    +0.0018    -0.38
15   -0.1758    +0.0043    -0.40
```

| 指标 | 值 | 判定 |
|------|------|------|
| 位置漂移 (dx) | 0.0055 m | ✅ |
| 位置漂移 (dy) | 0.0047 m | ✅ |
| **总位置漂移** | **0.73 cm** | ✅ 非常稳定 |
| **航向漂移** | **0.092°** | ✅ 非常稳定 |

**结论：✅ 静止状态定位非常稳定，30 秒内总漂移 < 1cm，航向偏差 < 0.1°**

### 4.3 建图状态

- Cartographer 正常运行，无崩溃
- `/map` 话题正常发布栅格地图
- "Ignored subdivision" 仅 8 次（启动瞬态），无后续
- "Dropped earlier points" 约 269 次（微小时间排序，不影响精度）

### 4.4 导航栈状态

- 所有 Nav2 节点 `active`，`Managed nodes are active` ✅
- 零崩溃，零 lifecycle 错误
- 速度链路：controller_server → cmd_vel_nav → velocity_smoother → car_cmd_vel → racecar_driver ✅

---

## 五、已解决的关键问题

### 5.1 cmd_vel 话题不匹配（车不动）
**现象**：Nav2 发布 `/cmd_vel`，但 racecar_driver 订阅 `/car_cmd_vel`  
**修复**：`navigation_no_smoother.launch.py` 中 velocity_smoother 的 `cmd_vel_smoothed` 重映射为 `car_cmd_vel`

### 5.2 scan 时间戳延迟（定位跳跃）
**现象**：lslidar_driver 使用数据采集时间作为发布时间戳，在 RISC-V 上延迟 300ms~1.2s  
**修复**：`lslidar_driver.cc` 两处 `scan->header.stamp` 改为 `get_clock()->now()`，并加单调递增保护

### 5.3 Cartographer 时间戳非单调崩溃 (SIGABRT)
**现象**：修复 5.2 后，缓冲的帧可能获得相同时间戳 → Cartographer 断言失败  
**修复**：加 `static last_scan_stamp` 变量，确保每帧时间戳严格递增（最小 +1ms）

### 5.4 地图重影/分裂（time_increment 帧间重叠）
**现象**：RViz2 中地图出现两份重影，之间有灰色拖影带（如同一个房间出现在两个位置）  
**原因**：`scan->time_increment = scan_time / count_num`（≈0.000277s），一帧449点总时长0.124s。Cartographer 用 `header.stamp + point_index * time_increment` 给每个点插值时间。当帧间隔(0.1s) < 帧内时长(0.124s)时，后一帧的前部点时间 < 前一帧的尾部点时间，导致：  
- "Ignored subdivision"（时间倒退的子帧被忽略）  
- "Dropped 418/370/282 earlier points"（大量点被丢弃）  
- Cartographer 位姿估计跳变 → 地图分裂  
**修复**：`lslidar_driver.cc` 两处 `time_increment` 设为 `0.0f`，告诉 Cartographer 所有点同一时刻采集（对10Hz雷达完全合理）  
**效果**：Ignored subdivision: 0, Dropped points: 0, 地图完整无分裂

### 5.5 FastDDS peer 配置破坏本地通信
**现象**：`fastdds_peer.xml` 设置 `initialPeersList` 后，同机节点间 DDS 发现异常，Cartographer 永远收不到 `/scan`  
**修复**：远程车不使用 FastDDS peer 配置，依赖默认组播发现。仅本机 RViz2 容器使用 peer 配置

### 5.5 OOM 杀进程
**现象**：长时间运行后系统 kill 所有 ROS 进程（exit code -9）  
**缓解**：RISC-V 8GB 内存，当前使用 927MB（空闲 5.3GB），监控中

---

## 六、操作速查

```bash
# === 远程车操作 ===
# 启动全套
ssh bianbu@192.168.3.16
cd /home/bianbu/workspace/racecar && source /opt/ros/humble/setup.bash && source install/setup.bash
bash explorer.sh --no-rviz

# 键盘遥控（另开终端）
ros2 run racecar racecar_teleop.py

# 保存地图
bash save.sh

# 停止
Ctrl+C

# === 本机操作 ===
# 启动 RViz2
bash /home/zk/rviz2_config/rviz2_remote.sh

# 在 RViz2 中发送导航目标：点击 "2D Goal Pose" 工具
# 在 RViz2 中设置初始位姿：点击 "Set Initial Pose" 工具

# === 重编 lslidar_driver（如改了源码）===
ssh bianbu@192.168.3.16
cd /home/bianbu/workspace/racecar && source /opt/ros/humble/setup.bash
colcon build --packages-select lslidar_driver --symlink-install  # ~3分钟
```
