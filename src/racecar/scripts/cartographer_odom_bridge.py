#!/usr/bin/env python3
"""Publish Cartographer's odom TF as nav_msgs/Odometry for Nav2."""

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class CartographerOdomBridge(Node):
    def __init__(self):
        super().__init__("cartographer_odom_bridge")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("odom_topic", "/odom_cartographer")
        self.declare_parameter("publish_rate", 20.0)
        self.declare_parameter("velocity_filter_gain", 0.35)

        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        odom_topic = self.get_parameter("odom_topic").value
        publish_rate = float(self.get_parameter("publish_rate").value)
        self.filter_gain = float(
            self.get_parameter("velocity_filter_gain").value
        )

        self.tf_buffer = Buffer(cache_time=Duration(seconds=2.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.publisher = self.create_publisher(Odometry, odom_topic, 10)
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_odom)

        self.previous = None
        self.filtered_vx = 0.0
        self.filtered_vy = 0.0
        self.filtered_wz = 0.0

    def publish_odom(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.odom_frame, self.base_frame, Time()
            )
        except TransformException as error:
            self.get_logger().warn(
                f"Waiting for {self.odom_frame}->{self.base_frame}: {error}",
                throttle_duration_sec=2.0,
            )
            return

        stamp_ns = (
            transform.header.stamp.sec * 1_000_000_000
            + transform.header.stamp.nanosec
        )
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        yaw = math.atan2(
            2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
            1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z),
        )

        if self.previous is not None:
            old_stamp_ns, old_x, old_y, old_yaw = self.previous
            dt = (stamp_ns - old_stamp_ns) / 1_000_000_000.0
            if 0.001 <= dt <= 0.5:
                dx = translation.x - old_x
                dy = translation.y - old_y
                world_vx = dx / dt
                world_vy = dy / dt
                body_vx = math.cos(yaw) * world_vx + math.sin(yaw) * world_vy
                body_vy = -math.sin(yaw) * world_vx + math.cos(yaw) * world_vy
                wz = wrap_angle(yaw - old_yaw) / dt
                gain = self.filter_gain
                self.filtered_vx += gain * (body_vx - self.filtered_vx)
                self.filtered_vy += gain * (body_vy - self.filtered_vy)
                self.filtered_wz += gain * (wz - self.filtered_wz)

        self.previous = (stamp_ns, translation.x, translation.y, yaw)

        message = Odometry()
        message.header = transform.header
        message.header.frame_id = self.odom_frame
        message.child_frame_id = self.base_frame
        message.pose.pose.position.x = translation.x
        message.pose.pose.position.y = translation.y
        message.pose.pose.position.z = translation.z
        message.pose.pose.orientation = rotation
        message.twist.twist.linear.x = self.filtered_vx
        message.twist.twist.linear.y = self.filtered_vy
        message.twist.twist.angular.z = self.filtered_wz
        message.pose.covariance[0] = 0.02
        message.pose.covariance[7] = 0.02
        message.pose.covariance[35] = 0.01
        message.twist.covariance[0] = 0.05
        message.twist.covariance[7] = 0.05
        message.twist.covariance[35] = 0.02
        self.publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = CartographerOdomBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
