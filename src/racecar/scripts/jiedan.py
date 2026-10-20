#!/usr/bin/env python3

"""一次发送全部 CSV 点位，并在指定点位停车一段时间。"""

import csv
import math
from collections import deque
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses
from nav2_msgs.msg import SpeedLimit
from rcl_interfaces.msg import Log
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


DEFAULT_CSV_PATH = '/home/bianbu/racecar/src/racecar/scripts/out_test.csv'

# ======================== 比赛停车点配置 ========================
# 点位编号从 1 开始，对应 CSV 的第 1 行、第 2 行……
# 例如要在第 3、10 个点位各停车一次，就改成 (3, 10)。
STOP_WAYPOINTS = (300, 500)

# 每个停车点的停车时间，单位：秒。
STOP_DURATION = 5.0

# 车辆距离停车点小于该数值时开始停车，单位：米。
STOP_TRIGGER_RADIUS = 0.50

# Nav2 用 0.0 表示“解除限速”，不能拿来表示停车。
# 0.001 m/s 经底层换算后仍是 1500 中位 PWM，因此车辆会停止。
STOP_SPEED_LIMIT = 0.001
# ===============================================================

ROS_LOG_WARN = 30
ROS_LOG_ERROR = 40
ROS_LOG_FATAL = 50


def read_waypoints_from_csv(filename):
    waypoints = []
    csv_path = Path(filename)
    if not csv_path.is_file():
        raise FileNotFoundError(f'点位文件不存在：{csv_path}')

    with csv_path.open('r', encoding='utf-8', newline='') as csv_file:
        for line_number, row in enumerate(csv.reader(csv_file), start=1):
            if not row or all(not value.strip() for value in row):
                continue
            if len(row) != 4:
                raise ValueError(
                    f'CSV 第 {line_number} 行必须是 x,y,qz,qw 四列，'
                    f'当前为 {len(row)} 列')

            try:
                x, y, qz, qw = (
                    float(value.strip()) for value in row)
            except ValueError as exc:
                raise ValueError(
                    f'CSV 第 {line_number} 行包含非数字内容') from exc

            if not all(math.isfinite(value) for value in (x, y, qz, qw)):
                raise ValueError(
                    f'CSV 第 {line_number} 行包含 NaN 或无穷大')

            quaternion_norm = math.hypot(qz, qw)
            if quaternion_norm < 1.0e-6:
                raise ValueError(
                    f'CSV 第 {line_number} 行的朝向四元数无效')

            waypoints.append(
                (x, y, qz / quaternion_norm, qw / quaternion_norm))

    if not waypoints:
        raise ValueError(f'点位文件为空：{csv_path}')
    return waypoints


def validate_stop_waypoints(stop_waypoints, waypoint_count):
    stop_points = tuple(int(point) for point in stop_waypoints)
    if len(stop_points) != 2:
        raise ValueError('STOP_WAYPOINTS 必须正好填写两个停车点位')
    if stop_points != tuple(sorted(set(stop_points))):
        raise ValueError('两个停车点位不能重复，并且必须从小到大填写')
    for point in stop_points:
        if not 1 <= point <= waypoint_count:
            raise ValueError(
                f'停车点位 {point} 超出 CSV 范围 1..{waypoint_count}')
    return stop_points


class NavThroughPosesClient(Node):
    def __init__(self):
        super().__init__('nav_through_poses_client')
        self.declare_parameter('csv_path', DEFAULT_CSV_PATH)

        self._action_client = ActionClient(
            self, NavigateThroughPoses, 'navigate_through_poses')
        self._speed_limit_publisher = self.create_publisher(
            SpeedLimit, 'speed_limit', 10)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._rosout_subscription = self.create_subscription(
            Log, '/rosout', self.rosout_callback, 100)
        self._control_timer = self.create_timer(0.05, self.control_timer_callback)

        self._goal_handle = None
        self._done = False
        self._waypoint_count = 0
        self._stop_waypoints = ()
        self._stop_targets = ()
        self._next_stop_index = 0
        self._pause_until_ns = 0
        self._paused_at_waypoint = None
        self._navigation_succeeded = False
        self._distance_remaining = float('nan')
        self._last_position_log_ns = 0
        self._last_tf_warning_ns = 0

        self._last_feedback_log_ns = 0
        self._collect_nav_logs = False
        self._diagnostic_logs = deque(maxlen=40)
        self._abort_report_timer = None

    @property
    def done(self):
        return self._done

    def start(self):
        csv_path = self.get_parameter(
            'csv_path').get_parameter_value().string_value
        try:
            waypoints = read_waypoints_from_csv(csv_path)
            self._stop_waypoints = validate_stop_waypoints(
                STOP_WAYPOINTS, len(waypoints))
        except (OSError, TypeError, ValueError) as exc:
            self.get_logger().error(str(exc))
            self._done = True
            return

        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(
                'NavigateThroughPoses 服务不可用，请确认 nav_one.sh 已正常启动')
            self._done = True
            return

        self._waypoint_count = len(waypoints)
        self._stop_targets = tuple(
            (
                point_number,
                waypoints[point_number - 1][0],
                waypoints[point_number - 1][1],
            )
            for point_number in self._stop_waypoints
        )
        goal = NavigateThroughPoses.Goal()
        stamp = self.get_clock().now().to_msg()
        for x, y, qz, qw in waypoints:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = stamp
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw
            goal.poses.append(pose)

        # 清除上一次程序异常退出时可能遗留的限速状态。
        self.publish_speed_limit(0.0)
        self.get_logger().info(
            f'一次发送全部 {self._waypoint_count} 个点位；'
            f'将在第 {self._stop_waypoints[0]}、'
            f'{self._stop_waypoints[1]} 个点位各停车 {STOP_DURATION:.1f} 秒')

        self._diagnostic_logs.clear()
        self._collect_nav_logs = True
        future = self._action_client.send_goal_async(
            goal, feedback_callback=self.feedback_callback)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        try:
            self._goal_handle = future.result()
        except Exception as exc:
            self.get_logger().error(f'发送路线失败：{exc}')
            self._done = True
            return

        if not self._goal_handle.accepted:
            self.get_logger().error('全部点位路线被 Nav2 拒绝')
            self._done = True
            return

        self.get_logger().info('全部点位路线已接受，L1 控制器开始跟踪')
        result_future = self._goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_message):
        feedback = feedback_message.feedback
        self._distance_remaining = getattr(
            feedback, 'distance_remaining', float('nan'))

    def control_timer_callback(self):
        now_ns = self.get_clock().now().nanoseconds

        if self._paused_at_waypoint is not None:
            if now_ns < self._pause_until_ns:
                return

            stopped_waypoint = self._paused_at_waypoint
            self._paused_at_waypoint = None
            self._pause_until_ns = 0
            self.publish_speed_limit(0.0)
            self.get_logger().info(
                f'第 {stopped_waypoint} 个点位停车结束，继续导航')
            if self._navigation_succeeded:
                self.finish_successfully()
            return

        if self._done or self._next_stop_index >= len(self._stop_targets):
            return

        try:
            transform = self._tf_buffer.lookup_transform(
                'map', 'base_footprint', Time())
        except TransformException as exc:
            if now_ns - self._last_tf_warning_ns >= 2_000_000_000:
                self._last_tf_warning_ns = now_ns
                self.get_logger().warn(
                    f'暂时无法获取车辆在 map 下的位置：{exc}')
            return

        stop_waypoint, target_x, target_y = self._stop_targets[
            self._next_stop_index]
        robot_x = transform.transform.translation.x
        robot_y = transform.transform.translation.y
        distance_to_stop = math.hypot(
            robot_x - target_x, robot_y - target_y)

        if now_ns - self._last_position_log_ns >= 1_000_000_000:
            self._last_position_log_ns = now_ns
            self.get_logger().info(
                f'剩余路线：{self._distance_remaining:.2f} 米，'
                f'距离第 {stop_waypoint} 个停车点：{distance_to_stop:.2f} 米')

        if distance_to_stop > STOP_TRIGGER_RADIUS:
            return

        self._next_stop_index += 1
        self._paused_at_waypoint = stop_waypoint
        self._pause_until_ns = now_ns + int(STOP_DURATION * 1.0e9)
        self.publish_speed_limit(STOP_SPEED_LIMIT)
        self.get_logger().warn(
            f'车辆进入 CSV 第 {stop_waypoint} 个点位的 '
            f'{STOP_TRIGGER_RADIUS:.2f} 米范围，停车 {STOP_DURATION:.1f} 秒')

    def result_callback(self, future):
        try:
            wrapped_result = future.result()
        except Exception as exc:
            self.publish_speed_limit(0.0)
            self.get_logger().error(f'导航任务失败：{exc}')
            self._done = True
            return

        self._goal_handle = None
        if wrapped_result.status == GoalStatus.STATUS_SUCCEEDED:
            self._collect_nav_logs = False
            self._navigation_succeeded = True
            if self._paused_at_waypoint is None:
                if self._next_stop_index < len(self._stop_targets):
                    missed = [
                        target[0]
                        for target in self._stop_targets[self._next_stop_index:]
                    ]
                    self.get_logger().warn(
                        f'导航已经结束，但停车点 {missed} 未进入 '
                        f'{STOP_TRIGGER_RADIUS:.2f} 米触发范围')
                self.finish_successfully()
        elif wrapped_result.status == GoalStatus.STATUS_CANCELED:
            self.publish_speed_limit(0.0)
            self.get_logger().warn('导航任务已取消')
            self._collect_nav_logs = False
            self._done = True
        else:
            self.publish_speed_limit(0.0)
            self.get_logger().error(
                f'导航任务异常结束，状态码：{wrapped_result.status}')
            self._abort_report_timer = self.create_timer(
                0.8, self.report_abort)

    def finish_successfully(self):
        self.publish_speed_limit(0.0)
        self.get_logger().info('全部点位导航完成')
        self._collect_nav_logs = False
        self._done = True

    def publish_speed_limit(self, speed_limit):
        message = SpeedLimit()
        message.header.stamp = self.get_clock().now().to_msg()
        message.percentage = False
        message.speed_limit = float(speed_limit)
        self._speed_limit_publisher.publish(message)

    def rosout_callback(self, message):
        if not self._collect_nav_logs or message.level < ROS_LOG_WARN:
            return
        if message.name == self.get_name():
            return

        relevant_names = (
            'bt_navigator',
            'planner_server',
            'controller_server',
            'global_costmap',
            'local_costmap',
            'lifecycle_manager_navigation',
        )
        if any(name in message.name for name in relevant_names):
            level = {
                ROS_LOG_WARN: 'WARN',
                ROS_LOG_ERROR: 'ERROR',
                ROS_LOG_FATAL: 'FATAL',
            }.get(message.level, str(message.level))
            self._diagnostic_logs.append(
                f'[{level}] [{message.name}] {message.msg}')

    def report_abort(self):
        if self._abort_report_timer is not None:
            self._abort_report_timer.cancel()
        self._collect_nav_logs = False

        if self._diagnostic_logs:
            self.get_logger().error('最近的 Nav2 警告或错误：')
            for line in self._diagnostic_logs:
                print(line, flush=True)
        else:
            self.get_logger().error(
                '没有收到 Nav2 警告，请确认 nav_one.sh 仍在运行')
        self._done = True

    def cancel(self):
        self.publish_speed_limit(0.0)
        if self._goal_handle is None:
            return None
        self.get_logger().warn('正在取消全部点位导航')
        return self._goal_handle.cancel_goal_async()


def main(args=None):
    rclpy.init(args=args)
    node = NavThroughPosesClient()
    node.start()

    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        cancel_future = node.cancel()
        if cancel_future is not None and rclpy.ok():
            rclpy.spin_until_future_complete(
                node, cancel_future, timeout_sec=2.0)
    finally:
        node.publish_speed_limit(0.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
