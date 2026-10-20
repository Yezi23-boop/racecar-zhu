#!/usr/bin/env python3

import csv
import math
from collections import deque

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rcl_interfaces.msg import Log


WAYPOINT_FILE = '/home/bianbu/racecar/src/racecar/scripts/out_test.csv'
ROS_LOG_WARN = 30
ROS_LOG_ERROR = 40
ROS_LOG_FATAL = 50


class SequentialNavClient(Node):
    """Send CSV waypoints as independent NavigateToPose goals."""

    def __init__(self, poses):
        super().__init__('sequential_nav_client')
        self.poses = poses
        self.current_index = 0
        self.goal_in_progress = False
        self.goal_handle = None
        self.done = False
        self.collect_nav_logs = False
        self.diagnostic_logs = deque(maxlen=40)
        self.abort_report_timer = None

        self.action_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose'
        )
        self.rosout_subscription = self.create_subscription(
            Log, '/rosout', self.rosout_callback, 100
        )

    def start(self):
        if not self.poses:
            self.get_logger().error('No valid waypoints found')
            self.done = True
            return

        self.get_logger().info('Waiting for Nav2 to become ready...')
        if not self.action_client.wait_for_server(timeout_sec=60.0):
            self.get_logger().error(
                'NavigateToPose action server unavailable after 60 seconds; '
                'check whether nav_one.sh started successfully')
            self.done = True
            return

        self.send_next_goal()

    def send_next_goal(self):
        if self.goal_in_progress:
            return

        if self.current_index >= len(self.poses):
            self.get_logger().info('All waypoints completed')
            self.done = True
            return

        pose = self.poses[self.current_index]
        pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        point_number = self.current_index + 1
        self.get_logger().info(
            f'Sending waypoint {point_number}/{len(self.poses)}: '
            f'x={pose.pose.position.x:.3f}, y={pose.pose.position.y:.3f}'
        )

        self.diagnostic_logs.clear()
        self.collect_nav_logs = True
        self.goal_in_progress = True
        send_future = self.action_client.send_goal_async(goal_msg)
        send_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        try:
            self.goal_handle = future.result()
        except Exception as exc:  # noqa: BLE001 - keep the vehicle stopped on failure
            self.goal_in_progress = False
            self.get_logger().error(f'Failed to send waypoint: {exc}')
            self.done = True
            return

        if not self.goal_handle.accepted:
            self.goal_in_progress = False
            self.get_logger().error(
                f'Waypoint {self.current_index + 1} rejected; sequence stopped'
            )
            self.done = True
            return

        self.get_logger().info(
            f'Waypoint {self.current_index + 1} accepted; L1 controller is following')
        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        try:
            wrapped_result = future.result()
        except Exception as exc:  # noqa: BLE001 - keep the vehicle stopped on failure
            self.goal_in_progress = False
            self.get_logger().error(f'Navigation result failed: {exc}')
            self.done = True
            return

        self.goal_in_progress = False
        point_number = self.current_index + 1

        if wrapped_result.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error(
                f'Waypoint {point_number} failed with status '
                f'{wrapped_result.status}; sequence stopped'
            )
            self.abort_report_timer = self.create_timer(0.8, self.report_abort)
            return

        self.collect_nav_logs = False
        self.get_logger().info(f'Waypoint {point_number} reached')
        self.current_index += 1
        self.send_next_goal()

    def rosout_callback(self, message):
        if not self.collect_nav_logs or message.level < ROS_LOG_WARN:
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
            self.diagnostic_logs.append(
                f'[{level}] [{message.name}] {message.msg}')

    def report_abort(self):
        if self.abort_report_timer is not None:
            self.abort_report_timer.cancel()
        self.collect_nav_logs = False

        if self.diagnostic_logs:
            self.get_logger().error('Recent Nav2 warnings/errors:')
            for line in self.diagnostic_logs:
                print(line, flush=True)
        else:
            self.get_logger().error(
                'No Nav2 warning reached /rosout; restart nav_one.sh before retrying')
        self.done = True

    def cancel(self):
        if self.goal_handle is None:
            return None
        self.get_logger().warn(
            f'Canceling waypoint {self.current_index + 1} and stopping navigation')
        return self.goal_handle.cancel_goal_async()


def read_poses_from_csv(filename):
    poses = []
    with open(filename, 'r', encoding='utf-8') as waypoint_file:
        reader = csv.reader(waypoint_file)
        for line_number, row in enumerate(reader, start=1):
            if not row:
                continue
            if len(row) != 4:
                raise ValueError(
                    f'Line {line_number}: expected 4 values, got {len(row)}'
                )

            x, y, z, w = (float(value.strip()) for value in row)
            if not all(math.isfinite(value) for value in (x, y, z, w)):
                raise ValueError(f'Line {line_number}: contains NaN or infinity')
            quaternion_norm = math.hypot(z, w)
            if quaternion_norm < 1.0e-6:
                raise ValueError(f'Line {line_number}: invalid orientation')
            z /= quaternion_norm
            w /= quaternion_norm

            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = z
            pose.pose.orientation.w = w
            poses.append(pose)

    return poses


def main(args=None):
    rclpy.init(args=args)

    try:
        poses = read_poses_from_csv(WAYPOINT_FILE)
    except (OSError, ValueError) as exc:
        node = rclpy.create_node('sequential_nav_client_error')
        node.get_logger().error(f'Cannot load waypoints: {exc}')
        node.destroy_node()
        rclpy.shutdown()
        return

    client = SequentialNavClient(poses)
    client.start()

    try:
        while rclpy.ok() and not client.done:
            rclpy.spin_once(client, timeout_sec=0.1)
    except KeyboardInterrupt:
        cancel_future = client.cancel()
        if cancel_future is not None and rclpy.ok():
            rclpy.spin_until_future_complete(client, cancel_future, timeout_sec=2.0)
    finally:
        client.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
