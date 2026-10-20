# 🧪 racecar Cartographer 探索方案 — 移植验证测试报告

> **生成日期：** 2026-03-22
> **目标平台：** bianbu (riscv64) @ 192.168.3.16
> **ROS 版本：** ROS 2 Humble
> **测试方式：** SSH 远程静态验证（包/节点/配置/TF链路）

---

## 📋 测试总览


| # | 功能模块                 | 状态    | 说明                                                   |
| - | ------------------------ | ------- | ------------------------------------------------------ |
| 1 | 遥控 (teleop)            | ✅ 通过 | `racecar_teleop.py` 可用，发布 `/cmd_vel`              |
| 2 | 定位 (Cartographer SLAM) | ✅ 通过 | `cartographer_node` 已安装，lua 配置正确               |
| 3 | 建图 (occupancy grid)    | ✅ 通过 | `cartographer_occupancy_grid_node` 已安装，发布 `/map` |
| 4 | 导航 (Nav2)              | ✅ 通过 | `navigation_launch.py` + `nav_carto.yaml` 配置完整     |

---

## 1️⃣ 遥控功能验证

### 可执行文件

```
racecar racecar_teleop.py  ✅
```

### 验证项


| 检查项                                   | 结果 |
| ---------------------------------------- | ---- |
| `racecar_teleop.py` 存在                 | ✅   |
| 发布话题`/cmd_vel` (geometry_msgs/Twist) | ✅   |
| 底盘驱动`racecar_driver_node` 存在       | ✅   |
| 底盘驱动`racecar_driver_node_one` 存在   | ✅   |

### 遥控测试命令

```bash
# 终端1: 启动底盘+传感器
bash explorer.sh


cd workspace/racecar/
source install/setup.bash
# 终端2: 启动遥控
ros2 run racecar racecar_teleop.py
```

---

## 2️⃣ 定位功能验证 (Cartographer SLAM)

### 可执行文件

```
cartographer_ros cartographer_node  ✅
```

### TF 链路设计

```
map ──(Cartographer发布)──> odom ──(rf2o发布)──> base_footprint ──> laser_link
                                                                 ──> IMU_link
```

### Cartographer 配置 (`racecar_2d.lua`)


| 参数                                   | 值           | 验证                                |
| -------------------------------------- | ------------ | ----------------------------------- |
| `tracking_frame`                       | `"IMU_link"` | ✅ 匹配实车 IMU frame_id            |
| `published_frame`                      | `"odom"`     | ✅ 发布 map→odom TF                |
| `provide_odom_frame`                   | `false`      | ✅ rf2o 已提供 odom→base_footprint |
| `use_odometry`                         | `false`      | ✅ 不依赖轮速里程计                 |
| `use_imu_data`                         | `true`       | ✅ 融合 IMU 数据                    |
| `use_online_correlative_scan_matching` | `true`       | ✅ 纯激光匹配                       |

### IMU 话题映射


| 原始话题                        | Cartographer 期望 | remap                  | 验证 |
| ------------------------------- | ----------------- | ---------------------- | ---- |
| `/IMU_data` (frame_id=IMU_link) | `/imu`            | `('imu', '/IMU_data')` | ✅   |
| `/scan`                         | `/scan`           | 无需 remap             | ✅   |

### EKF 融合配置 (`ekf_carto.yaml`)


| 参数              | 值               | 验证                   |
| ----------------- | ---------------- | ---------------------- |
| `odom_frame`      | `odom`           | ✅                     |
| `world_frame`     | `odom`           | ✅                     |
| `base_link_frame` | `base_footprint` | ✅                     |
| `odom0`           | `odom_rf2o`      | ✅ rf2o 激光里程计话题 |
| `imu0`            | `IMU_data`       | ✅ 实车 IMU 话题       |
| `publish_tf`      | `false`          | ✅ 避免与 rf2o 冲突    |

### 静态 TF


| 父坐标系         | 子坐标系     | 偏移 (x,y,z) | 验证 |
| ---------------- | ------------ | ------------ | ---- |
| `base_footprint` | `laser_link` | 0.07, 0, 0   | ✅   |
| `base_footprint` | `IMU_link`   | 0.1653, 0, 0 | ✅   |

### 定位测试命令

```bash
# 启动后检查 TF 树
ros2 run tf2_tools view_frames

# 检查 map→odom 变换是否存在
ros2 run tf2_ros tf2_echo map odom

# 检查 odom→base_footprint 变换
ros2 run tf2_ros tf2_echo odom base_footprint
```

---

## 3️⃣ 建图功能验证 (Occupancy Grid)

### 可执行文件

```
cartographer_ros cartographer_occupancy_grid_node  ✅
```

### 验证项


| 检查项                                    | 结果 |
| ----------------------------------------- | ---- |
| `cartographer_occupancy_grid_node` 已安装 | ✅   |
| 分辨率设置 0.05m                          | ✅   |
| 发布周期 1.0s                             | ✅   |
| 发布到`/map` 话题                         | ✅   |

### 建图测试命令

```bash
# 启动后检查 /map 话题
ros2 topic echo /map --once | head -10

# 检查子图数量
ros2 topic echo /submap_list --once | head -5

# 保存地图
ros2 run nav2_map_server map_saver_cli -f ~/map_test
```

### 保存地图（退出时自动保存）

```bash
bash explorer.sh --save-map
# Ctrl+C 退出时自动保存到 src/racecar/map/carto_map_<timestamp>.pgm/.yaml/.pbstream
```

---

## 4️⃣ 导航功能验证 (Nav2)

### 包依赖


| 包                                       | 安装路径          | 验证 |
| ---------------------------------------- | ----------------- | ---- |
| `nav2_bringup`                           | `/opt/ros/humble` | ✅   |
| `nav2_regulated_pure_pursuit_controller` | `/opt/ros/humble` | ✅   |
| `nav2_smac_planner`                      | `/opt/ros/humble` | ✅   |
| `nav2_behaviors`                         | `/opt/ros/humble` | ✅   |
| `nav2_costmap_2d`                        | `/opt/ros/humble` | ✅   |

### Nav2 参数 (`nav_carto.yaml`) 关键配置


| 模块              | 参数               | 值                               | 验证 |
| ----------------- | ------------------ | -------------------------------- | ---- |
| AMCL              | 状态               | 已禁用 (`amcl_disabled`)         | ✅   |
| bt_navigator      | `odom_topic`       | `odom`                           | ✅   |
| bt_navigator      | `robot_base_frame` | `base_footprint`                 | ✅   |
| controller_server | `odom_topic`       | `odom`                           | ✅   |
| controller_server | 插件               | `RegulatedPurePursuitController` | ✅   |
| planner_server    | 插件               | `SmacPlannerHybrid`              | ✅   |
| local_costmap     | `global_frame`     | `odom`                           | ✅   |
| global_costmap    | `global_frame`     | `map`                            | ✅   |
| behavior_server   | `global_frame`     | `odom`                           | ✅   |
| 启动方式          | launcher           | `navigation_launch.py` (无AMCL)  | ✅   |

### 导航测试命令

```bash
# 启动后检查 Nav2 生命周期节点
ros2 lifecycle list /controller_server
ros2 lifecycle list /planner_server
ros2 lifecycle list /bt_navigator

# 发送导航目标
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 0.0}, orientation: {w: 1.0}}}}"
```

---

## 🏗️ 系统架构总览

```
┌────────────────────────────────────────────────────────────────────┐
│                    Run_explorer.launch.py                         │
├──────────────┬──────────────┬──────────────┬──────────────────────┤
│  第1层 (0s)  │  第2层 (0s)  │  第3层 (3s)  │   第4层 (8s)         │
│  硬件驱动    │  状态估计    │  SLAM定位    │   导航规划           │
├──────────────┼──────────────┼──────────────┼──────────────────────┤
│ racecar_drv  │ ekf_node     │ cartographer │ Nav2                 │
│ lslidar      │ (rf2o+IMU)   │   _node      │ navigation_launch.py │
│ imu_get      │              │ occupancy    │ (controller_server,  │
│ rf2o_odom    │              │   _grid_node │  planner_server,     │
│ static TFs   │              │              │  bt_navigator, ...)  │
│ joint_state  │              │              │                      │
└──────────────┴──────────────┴──────────────┴──────────────────────┘

话题流:
  /scan ──────────> rf2o ──> /odom_rf2o ──> EKF ──> /odom
  /scan ──────────> Cartographer ──────────> /map
  /IMU_data ──────> Cartographer (via remap)
  /IMU_data ──────> EKF (imu0)
  /cmd_vel <─────── Nav2 controller_server
  /cmd_vel <─────── racecar_teleop.py (手动遥控)

TF链路:
  map ──(Cartographer)──> odom ──(rf2o)──> base_footprint ──> laser_link
                                                            ──> IMU_link
```

---

## 📁 新增/修改文件清单


| 文件                                 | 类型 | 作用                                              |
| ------------------------------------ | ---- | ------------------------------------------------- |
| `config/cartographer/racecar_2d.lua` | 新增 | Cartographer 2D配置                               |
| `config/nav_carto.yaml`              | 新增 | Nav2参数(无AMCL)                                  |
| `config/ekf_carto.yaml`              | 修改 | EKF: odom→odom, odom0→odom_rf2o, imu0→IMU_data |
| `launch/cartographer.launch.py`      | 新增 | Cartographer+OccupancyGrid launch                 |
| `launch/Run_explorer.launch.py`      | 新增 | 一体化4层启动                                     |
| `explorer.sh`                        | 新增 | 一键启动脚本(支持--save-map)                      |
| `CMakeLists.txt`                     | 修改 | 注释find_package(cartographer_ros)                |
| `package.xml`                        | 修改 | 添加exec_depend cartographer_ros                  |

---

## 🚀 快速上车测试流程

```bash
# 1. SSH 到实车
ssh bianbu@192.168.3.16

# 2. 进入工作空间
cd /home/bianbu/workspace/racecar

# 3. 启动探索模式
bash explorer.sh

# 4. 另开终端，遥控验证
ssh bianbu@192.168.3.16
source /opt/ros/humble/setup.bash
source /home/bianbu/workspace/racecar/install/setup.bash
ros2 run racecar racecar_teleop.py

# 5. 另开终端，检查 TF 和话题
ros2 topic list
ros2 topic hz /scan
ros2 topic hz /IMU_data
ros2 topic hz /odom
ros2 topic hz /map
ros2 run tf2_ros tf2_echo map base_footprint

# 6. 发送导航目标测试
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 0.0}, orientation: {w: 1.0}}}}"

# 7. 退出时保存地图
# Ctrl+C explorer.sh (如果用了 --save-map 会自动保存)
```

---

## ⚠️ 已知注意事项

1. **延迟启动**：Cartographer 延迟 3s，Nav2 延迟 8s，等传感器就绪
2. **IMU 话题**：Cartographer 通过 remap `imu→/IMU_data` 获取 IMU 数据
3. **无轮速里程计**：完全依赖 rf2o 激光里程计 + IMU
4. **odom frame**：Cartographer 模式统一使用 `odom`（非原来的 `odom_combined`）
5. **原有 car.sh / nav.sh 不受影响**：使用原始 EKF+AMCL 方案时仍可照常使用
