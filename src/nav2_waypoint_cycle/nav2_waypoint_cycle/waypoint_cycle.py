import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Twist, PointStamped
from visualization_msgs.msg import Marker, MarkerArray
from nav2_msgs.action import NavigateToPose
import subprocess
import sys
import os
import math
import select
import termios
import tty
import threading


class waypoint_cycle(Node):

    def __init__(self):
        super().__init__("waypoint_cycle")
        self.declare_parameter("enable_loop", False)
        self.enable_loop = self.get_parameter("enable_loop").value

        self.waypoints = []
        self.markerArray = MarkerArray()
        self.markerArray_number = MarkerArray()
        self.count = 0
        self.index = 0
        self.try_again = True
        self.navigating = False
        self.finished = False
        self._goal_handle = None

        self.mark_pub = self.create_publisher(MarkerArray, "/path_point", 100)
        self.cmd_vel_pub = self.create_publisher(Twist, "/car_cmd_vel", 10)

        self._nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        # Publish Point 工具发布 /clicked_point
        self.create_subscription(PointStamped, "/clicked_point", self._clicked_cb, 10)

        self._keyboard_thread = threading.Thread(target=self._keyboard_listener, daemon=True)
        self._keyboard_thread.start()
        self._print_help()

    def _print_help(self):
        self.get_logger().info("========== 快捷键 ==========")
        self.get_logger().info("  RViz Publish Point 选点")
        self.get_logger().info("  r: 开始执行导航")
        self.get_logger().info("  c: 清除所有点 / 停止导航")
        self.get_logger().info("  f: 保存地图")
        self.get_logger().info("============================")

    def _keyboard_listener(self):
        try:
            old_settings = termios.tcgetattr(sys.stdin)
        except termios.error:
            self.get_logger().warn("无终端，键盘不可用")
            return
        try:
            tty.setcbreak(sys.stdin.fileno())
            while rclpy.ok():
                if select.select([sys.stdin], [], [], 0.2)[0]:
                    key = sys.stdin.read(1)
                    if key in ("r", "R"):
                        self._start_nav()
                    elif key in ("c", "C"):
                        self._clear()
                    elif key in ("f", "F"):
                        self._save_map()
        except Exception:
            pass
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

    def _start_nav(self):
        if self.navigating:
            self.get_logger().warn("导航中，按c停止")
            return
        if not self.waypoints:
            self.get_logger().warn("没有目标点！用 Publish Point 选点")
            return
        self.get_logger().info(">>> 开始导航 %d 个点 <<<" % len(self.waypoints))
        self.navigating = True
        self.finished = False
        self.index = 0
        self.try_again = True
        self._send_goal(self.index)

    def _clear(self):
        if self.navigating:
            self.get_logger().info("停止导航")
            self._cancel_goal()
            self._send_stop()
        self._delete_markers()
        self._reset()
        self.get_logger().info("已清除，可重新选点")

    def _reset(self):
        self.waypoints = []
        self.markerArray = MarkerArray()
        self.markerArray_number = MarkerArray()
        self.count = 0
        self.index = 0
        self.navigating = False
        self.finished = False
        self.try_again = True
        self._goal_handle = None

    def _save_map(self):
        try:
            d = os.path.expanduser("~/racecar/map")
            os.makedirs(d, exist_ok=True)
            import datetime
            t = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            p = "%s/map_%s" % (d, t)
            self.get_logger().info("保存地图: %s" % p)
            subprocess.Popen(["ros2","run","nav2_map_server","map_saver_cli","-f",p,"--ros-args","-p","save_map_timeout:=5.0"])
        except Exception as e:
            self.get_logger().error("失败: %s" % str(e))

    # ==================== /clicked_point 回调 ====================
    def _clicked_cb(self, msg):
        x = msg.point.x
        y = msg.point.y

        if self.navigating:
            self.get_logger().warn("导航中，按c停止后重选")
            return
        if self.finished:
            self.get_logger().info("已结束，按c清除后重选")
            return

        # 计算朝向: 从上一个点指向当前点；第一个点默认朝前(yaw=0)
        if self.waypoints:
            px, py = self.waypoints[-1][0], self.waypoints[-1][1]
            yaw = math.atan2(y - py, x - px)
        else:
            yaw = 0.0
        oz = math.sin(yaw / 2.0)
        ow = math.cos(yaw / 2.0)

        self.waypoints.append((x, y, oz, ow))
        self._add_marker(x, y, oz, ow)
        self._add_number(x, y, oz, ow)
        self.mark_pub.publish(self.markerArray)
        self.mark_pub.publish(self.markerArray_number)
        self.count = len(self.waypoints)
        self.get_logger().info("#%d: x=%.3f y=%.3f yaw=%.1f°" % (self.count, x, y, math.degrees(yaw)))
        self.get_logger().info("  共%d个 | r=开始 c=清除" % self.count)

    # ==================== Action Client ====================
    def _send_goal(self, idx):
        if idx >= len(self.waypoints):
            return
        wp = self.waypoints[idx]
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = wp[0]
        goal.pose.pose.position.y = wp[1]
        goal.pose.pose.orientation.z = wp[2]
        goal.pose.pose.orientation.w = wp[3]
        self.get_logger().info(">> 导航到 #%d: x=%.3f y=%.3f" % (idx + 1, wp[0], wp[1]))

        if not self._nav_client.server_is_ready():
            self.get_logger().warn("等待 navigate_to_pose server...")
            self._nav_client.wait_for_server(timeout_sec=5.0)

        future = self._nav_client.send_goal_async(goal)
        future.add_done_callback(self._goal_accepted_cb)

    def _goal_accepted_cb(self, future):
        gh = future.result()
        if not gh.accepted:
            self.get_logger().warn("目标被拒绝")
            return
        self._goal_handle = gh
        gh.get_result_async().add_done_callback(self._result_cb)

    def _result_cb(self, future):
        if not self.navigating or self.finished:
            return
        status = future.result().status

        if status == 4:  # SUCCEEDED
            self.get_logger().info("到达 #%d" % (self.index + 1))
            self.try_again = True
            self.index += 1
            if self.index >= self.count:
                if self.enable_loop:
                    self.get_logger().info("循环: 重新开始")
                    self.index = 0
                    self._send_goal(0)
                else:
                    self.get_logger().info(">>> 全部完成! <<<")
                    self.navigating = False
                    self.finished = True
            else:
                self._send_goal(self.index)
        elif status == 6:  # ABORTED
            self.get_logger().warn("#%d 失败" % (self.index + 1))
            if self.try_again:
                self.get_logger().warn("重试一次")
                self._send_goal(self.index)
                self.try_again = False
            else:
                self.index += 1
                self.try_again = True
                if self.index < len(self.waypoints):
                    self.get_logger().warn("跳过，去下一个")
                    self._send_goal(self.index)
                else:
                    self.get_logger().info(">>> 全部完成! <<<")
                    self.navigating = False
                    self.finished = True
        elif status == 5:  # CANCELED
            pass

    def _cancel_goal(self):
        try:
            if self._goal_handle is not None:
                self._goal_handle.cancel_goal_async()
        except Exception:
            pass

    # ==================== 辅助 ====================
    def _send_stop(self):
        t = Twist()
        t.linear.x = 0.0
        t.angular.z = 0.0
        self.cmd_vel_pub.publish(t)

    def _add_marker(self, x, y, oz, ow):
        m = Marker()
        m.header.frame_id = "map"
        m.ns = "arrow"
        m.id = self.count
        m.type = Marker.ARROW
        m.action = Marker.ADD
        m.scale.x = 0.5
        m.scale.y = 0.05
        m.scale.z = 0.05
        m.color.a = 1.0
        m.color.r = 1.0
        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = 0.1
        m.pose.orientation.z = oz
        m.pose.orientation.w = ow
        self.markerArray.markers.append(m)

    def _add_number(self, x, y, oz, ow):
        m = Marker()
        m.header.frame_id = "map"
        m.ns = "number"
        m.id = self.count
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD
        m.scale.x = 0.5
        m.scale.y = 0.5
        m.scale.z = 0.5
        m.color.a = 1.0
        m.color.r = 1.0
        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = 0.5
        m.pose.orientation.z = oz
        m.pose.orientation.w = ow
        m.text = str(self.count)
        self.markerArray_number.markers.append(m)

    def _delete_markers(self):
        da = MarkerArray()
        for m in self.markerArray.markers:
            d = Marker()
            d.header.frame_id = "map"
            d.ns = m.ns
            d.id = m.id
            d.action = Marker.DELETE
            da.markers.append(d)
        for m in self.markerArray_number.markers:
            d = Marker()
            d.header.frame_id = "map"
            d.ns = m.ns
            d.id = m.id
            d.action = Marker.DELETE
            da.markers.append(d)
        if da.markers:
            self.mark_pub.publish(da)


def main(args=None):
    rclpy.init(args=args)
    node = waypoint_cycle()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, Exception):
        pass
    finally:
        try:
            node._send_stop()
            node.get_logger().info("停车")
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
