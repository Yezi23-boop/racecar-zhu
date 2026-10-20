#!/usr/bin/env python3

import csv
import json
import math
from collections import deque
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateThroughPoses
from rclpy.action import ActionClient
from rclpy.node import Node
from rcl_interfaces.msg import Log
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


# 默认导航点 CSV 文件路径，文件内容由 get.py 采点生成。
DEFAULT_CSV_PATH = '/home/bianbu/racecar/src/racecar/scripts/out_test.csv'
# 道路感知话题，由 traffic_light_detector.py 发布红绿灯和人行道状态。
DEFAULT_PERCEPTION_TOPIC = '/road_perception'
# 检测到人行道后停车等待的时间，单位：秒。
CROSSWALK_STOP_DURATION = 3.0
# 停车时发给底盘控制话题的速度和转向值。
STOP_SPEED = 0.0
STOP_STEERING = 0.0
# 停车期间重复发布停车指令的周期，单位：秒。
STOP_PUBLISH_PERIOD = 0.1
# /rosout 日志等级常量，用于筛选 Nav2 的警告、错误和严重错误日志。
ROS_LOG_WARN = 30
ROS_LOG_ERROR = 40
ROS_LOG_FATAL = 50


def read_waypoints_from_csv(filename):
    waypoints = []
    csv_path = Path(filename)
    if not csv_path.is_file():
        raise FileNotFoundError(f'Waypoint CSV does not exist: {csv_path}')

    with csv_path.open('r', encoding='utf-8', newline='') as csv_file:
        for line_number, row in enumerate(csv.reader(csv_file), start=1):
            if not row or all(not value.strip() for value in row):
                continue
            if len(row) != 4:
                raise ValueError(
                    f'CSV line {line_number} must contain x,y,qz,qw; got {len(row)} columns')

            try:
                x, y, qz, qw = (float(value.strip()) for value in row)
            except ValueError as exc:
                raise ValueError(f'CSV line {line_number} contains a non-numeric value') from exc

            if not all(math.isfinite(value) for value in (x, y, qz, qw)):
                raise ValueError(f'CSV line {line_number} contains NaN or infinity')

            quaternion_norm = math.hypot(qz, qw)
            if quaternion_norm < 1.0e-6:
                raise ValueError(f'CSV line {line_number} contains an invalid orientation')

            waypoints.append((x, y, qz / quaternion_norm, qw / quaternion_norm))

    if not waypoints:
        raise ValueError(f'Waypoint CSV is empty: {csv_path}')
    return waypoints


class NavThroughPosesClient(Node):
    def __init__(self):
        super().__init__('nav_through_poses_client')
        self.declare_parameter('csv_path', DEFAULT_CSV_PATH)
        self.declare_parameter('perception_topic', DEFAULT_PERCEPTION_TOPIC)
        self.declare_parameter(
            'crosswalk_stop_duration', CROSSWALK_STOP_DURATION)

        self._crosswalk_stop_duration = max(
            0.1,
            float(self.get_parameter('crosswalk_stop_duration').value),
        )
        self._action_client = ActionClient(
            self, NavigateThroughPoses, 'navigate_through_poses')
        self._goal_handle = None
        self._done = False
        self._last_feedback_log_ns = 0
        self._collect_nav_logs = False
        self._diagnostic_logs = deque(maxlen=40)
        self._abort_report_timer = None
        self._red_stop_active = False
        self._crosswalk_handled = False
        self._crosswalk_stop_until_ns = 0
        self._motion_stopped = False
        self._last_traffic_light = 'UNKNOWN'

        perception_qos = QoSProfile(depth=1)
        perception_qos.reliability = ReliabilityPolicy.RELIABLE
        perception_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        perception_topic = self.get_parameter(
            'perception_topic').get_parameter_value().string_value
        self._perception_subscription = self.create_subscription(
            String,
            perception_topic,
            self.perception_callback,
            perception_qos,
        )
        self._cmd_vel_subscription = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self._car_cmd_vel_publisher = self.create_publisher(
            Twist, '/car_cmd_vel', 10)
        self._traffic_control_timer = self.create_timer(
            0.1, self.traffic_control_timer_callback)
        self._stop_timer = self.create_timer(
            STOP_PUBLISH_PERIOD, self.publish_stop_if_needed)
        self._rosout_subscription = self.create_subscription(
            Log, '/rosout', self.rosout_callback, 100)

        self.get_logger().info(
            f'Listening for traffic control on {perception_topic}')
        self.get_logger().info('Forwarding /cmd_vel to /car_cmd_vel')

    @property
    def done(self):
        return self._done

    def perception_callback(self, message):
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError) as exc:
            self.get_logger().warn(
                f'Ignoring invalid road perception message: {exc}')
            return
        if not isinstance(payload, dict):
            self.get_logger().warn(
                'Ignoring road perception message that is not a JSON object')
            return

        traffic_light = str(
            payload.get('traffic_light', 'UNKNOWN')).upper()
        crosswalk = payload.get('crosswalk', False)
        if traffic_light not in ('RED', 'GREEN', 'UNKNOWN'):
            self.get_logger().warn(
                f'Ignoring unsupported traffic-light state: {traffic_light}')
            traffic_light = 'UNKNOWN'
        if not isinstance(crosswalk, bool):
            self.get_logger().warn(
                'Ignoring non-boolean crosswalk state')
            crosswalk = False

        if traffic_light != self._last_traffic_light:
            self.get_logger().info(
                f'Traffic light changed to {traffic_light}')
            self._last_traffic_light = traffic_light

        if traffic_light == 'RED':
            if not self._red_stop_active:
                self.get_logger().warn('Red light detected: stopping')
            self._red_stop_active = True
        elif traffic_light == 'GREEN':
            if self._red_stop_active:
                self.get_logger().info('Green light detected: navigation may continue')
            self._red_stop_active = False

        now_ns = self.get_clock().now().nanoseconds
        if crosswalk and not self._crosswalk_handled:
            self._crosswalk_handled = True
            stop_duration_ns = int(self._crosswalk_stop_duration * 1.0e9)
            self._crosswalk_stop_until_ns = max(
                self._crosswalk_stop_until_ns,
                now_ns + stop_duration_ns,
            )
            self.get_logger().warn(
                f'Crosswalk detected: stopping for {self._crosswalk_stop_duration:.1f} s')
        self.update_motion_stop(force=True)

    def traffic_control_timer_callback(self):
        now_ns = self.get_clock().now().nanoseconds
        if (
            self._crosswalk_stop_until_ns > 0
            and now_ns >= self._crosswalk_stop_until_ns
        ):
            self._crosswalk_stop_until_ns = 0
            self.get_logger().info('Crosswalk stop completed')
        self.update_motion_stop()

    def update_motion_stop(self, force=False):
        now_ns = self.get_clock().now().nanoseconds
        crosswalk_stop_active = (
            self._crosswalk_stop_until_ns > now_ns)
        should_stop = self._red_stop_active or crosswalk_stop_active
        state_changed = should_stop != self._motion_stopped
        if state_changed:
            self._motion_stopped = should_stop
            if should_stop:
                reasons = []
                if self._red_stop_active:
                    reasons.append('red light')
                if crosswalk_stop_active:
                    reasons.append('crosswalk')
                self.get_logger().warn(
                    f'Navigation paused by {" and ".join(reasons)}')
            else:
                self.get_logger().info('Traffic control released: navigation resumed')

        if force or state_changed or should_stop:
            self.publish_stop_if_needed()

    def make_stop_twist(self):
        twist = Twist()
        twist.linear.x = STOP_SPEED
        twist.angular.z = STOP_STEERING
        return twist

    def publish_stop(self):
        self._car_cmd_vel_publisher.publish(self.make_stop_twist())

    def publish_stop_if_needed(self):
        if self._motion_stopped:
            self.publish_stop()

    def cmd_vel_callback(self, message):
        if self._motion_stopped:
            self.publish_stop()
            return

        self._car_cmd_vel_publisher.publish(message)

    def release_stop(self):
        self._red_stop_active = False
        self._crosswalk_stop_until_ns = 0
        self._motion_stopped = False
        self.publish_stop()

    def start(self):
        csv_path = self.get_parameter('csv_path').get_parameter_value().string_value
        try:
            waypoints = read_waypoints_from_csv(csv_path)
        except (OSError, ValueError) as exc:
            self.get_logger().error(str(exc))
            self._done = True
            return

        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('NavigateThroughPoses action server is unavailable')
            self._done = True
            return

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

        self.get_logger().info(
            f'Sending {len(goal.poses)} waypoints from {csv_path}')
        self._diagnostic_logs.clear()
        self._collect_nav_logs = True
        future = self._action_client.send_goal_async(
            goal, feedback_callback=self.feedback_callback)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        try:
            self._goal_handle = future.result()
        except Exception as exc:  # rclpy reports transport errors through the future
            self.get_logger().error(f'Failed to send waypoint goal: {exc}')
            self._done = True
            return

        if not self._goal_handle.accepted:
            self.get_logger().error('Waypoint goal was rejected')
            self._done = True
            return

        self.get_logger().info('Waypoint goal accepted; L1 controller is following the route')
        result_future = self._goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_message):
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self._last_feedback_log_ns < 1_000_000_000:
            return
        self._last_feedback_log_ns = now_ns

        feedback = feedback_message.feedback
        remaining = getattr(feedback, 'number_of_poses_remaining', -1)
        distance = getattr(feedback, 'distance_remaining', float('nan'))
        self.get_logger().info(
            f'Waypoints remaining: {remaining}, route distance remaining: {distance:.2f} m')

    def result_callback(self, future):
        try:
            wrapped_result = future.result()
        except Exception as exc:
            self.get_logger().error(f'Waypoint navigation failed: {exc}')
            self._done = True
            return

        if wrapped_result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('All waypoints completed')
            self._collect_nav_logs = False
            self._done = True
        elif wrapped_result.status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn('Waypoint navigation canceled')
            self._collect_nav_logs = False
            self._done = True
        else:
            self.get_logger().error(
                f'Waypoint navigation ended with status {wrapped_result.status}')
            # Give /rosout a short moment to deliver the planner / BT error that
            # normally arrives at nearly the same time as the action result.
            self._abort_report_timer = self.create_timer(0.8, self.report_abort)

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
            self.get_logger().error('Recent Nav2 warnings/errors:')
            for line in self._diagnostic_logs:
                print(line, flush=True)
        else:
            self.get_logger().error(
                'No Nav2 warning reached /rosout; check whether nav_one.sh is still running')
        self._done = True

    def cancel(self):
        if self._goal_handle is None:
            return None
        self.get_logger().warn('Canceling waypoint navigation and stopping the controller')
        return self._goal_handle.cancel_goal_async()


def main(args=None):
    rclpy.init(args=args)
    node = NavThroughPosesClient()
    # 先接收一次检测器保留的最新状态，避免红灯下发送导航目标后短暂起步。
    rclpy.spin_once(node, timeout_sec=0.5)
    node.start()

    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        cancel_future = node.cancel()
        if cancel_future is not None and rclpy.ok():
            rclpy.spin_until_future_complete(node, cancel_future, timeout_sec=2.0)
    finally:
        node.release_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
