#!/usr/bin/env python3

import math
from enum import Enum

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point, PoseStamped
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import Marker


class State(Enum):
    ROTATE = 'ROTATE_TO_GOAL'
    DRIVE = 'DRIVE_TO_GOAL'
    ALIGN = 'FINAL_ALIGN'
    DONE = 'DONE'


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def yaw_from_quat(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )


def slew(target, current, max_rate, dt):
    step = max_rate * dt
    return current + clamp(target - current, -step, step)


class WaypointNavigator(Node):

    def __init__(self):
        super().__init__('waypoint_navigator')

        self.declare_parameter('waypoints', [0.0, 0.0, 0.0])
        self.declare_parameter('pos_tolerance', 0.05)
        self.declare_parameter('yaw_tolerance_deg', 3.0)
        self.declare_parameter('align_tolerance_deg', 4.0)
        self.declare_parameter('realign_threshold_deg', 35.0)
        self.declare_parameter('k_linear', 0.8)
        self.declare_parameter('k_angular', 1.8)
        self.declare_parameter('max_linear', 0.20)
        self.declare_parameter('min_linear', 0.04)
        self.declare_parameter('max_angular', 1.0)
        self.declare_parameter('max_lin_accel', 0.4)
        self.declare_parameter('max_ang_accel', 2.5)
        self.declare_parameter('control_rate', 30.0)

        g = lambda n: self.get_parameter(n).value

        flat = list(g('waypoints'))

        if len(flat) % 3 != 0:
            raise ValueError(
                'waypoints must be [x, y, yaw_deg] triples'
            )

        self.wps = [
            (
                float(flat[i]),
                float(flat[i + 1]),
                math.radians(float(flat[i + 2]))
            )
            for i in range(0, len(flat), 3)
        ]

        self.pos_tol = float(g('pos_tolerance'))
        self.yaw_tol = math.radians(float(g('yaw_tolerance_deg')))
        self.align_tol = math.radians(float(g('align_tolerance_deg')))
        self.realign = math.radians(float(g('realign_threshold_deg')))

        self.k_lin = float(g('k_linear'))
        self.k_ang = float(g('k_angular'))

        self.v_max = float(g('max_linear'))
        self.v_min = float(g('min_linear'))
        self.w_max = float(g('max_angular'))

        self.a_lin = float(g('max_lin_accel'))
        self.a_ang = float(g('max_ang_accel'))

        self.dt = 1.0 / float(g('control_rate'))

        self.pose = None
        self.idx = 0
        self.state = State.ROTATE

        self.cmd_v = 0.0
        self.cmd_w = 0.0

        self.pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.marker_pub = self.create_publisher(
            Marker,
            '/visualization_marker',
            10
        )

        self.path_pub = self.create_publisher(
            Path,
            '/path',
            10
        )

        self.path = Path()
        self.path.header.frame_id = 'odom'

        self.create_subscription(
            Odometry,
            '/odom',
            self.odom_cb,
            10
        )

        self.create_timer(
            self.dt,
            self.loop
        )

        self.get_logger().info(
            f'Loaded {len(self.wps)} waypoints.'
        )

    def yaw_to_quaternion(self, yaw):
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        return qz, qw

    def publish_arena(self):
        marker = Marker()

        marker.header.frame_id = 'odom'
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = 'arena'
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        marker.scale.x = 0.025

        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        pts = [
            (-0.225, -0.850),
            (0.225, -0.850),
            (0.225, 0.850),
            (-0.225, 0.850),
            (-0.225, -0.850)
        ]

        for x, y in pts:
            p = Point()
            p.x = float(x)
            p.y = float(y)
            p.z = 0.015
            marker.points.append(p)

        self.marker_pub.publish(marker)

    def publish_safe_zone(self):
        marker = Marker()

        marker.header.frame_id = 'odom'
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = 'navigation_safe_zone'
        marker.id = 10
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        marker.scale.x = 0.018

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        pts = [
            (-0.1005, -0.6740),
            (0.1005, -0.6740),
            (0.1005, 0.6740),
            (-0.1005, 0.6740),
            (-0.1005, -0.6740)
        ]

        for x, y in pts:
            p = Point()
            p.x = float(x)
            p.y = float(y)
            p.z = 0.025
            marker.points.append(p)

        self.marker_pub.publish(marker)

    def publish_goal(self):
        marker = Marker()

        marker.header.frame_id = 'odom'
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = 'goal'
        marker.id = 11
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        marker.pose.position.x = 0.1005
        marker.pose.position.y = -0.6500
        marker.pose.position.z = 0.006

        marker.scale.x = 0.10
        marker.scale.y = 0.10
        marker.scale.z = 0.01

        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 0.8

        self.marker_pub.publish(marker)

    def publish_waypoints(self):
        marker = Marker()

        marker.header.frame_id = 'odom'
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = 'wps'
        marker.id = 1
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD

        marker.scale.x = 0.08
        marker.scale.y = 0.08
        marker.scale.z = 0.08

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        for wp in self.wps:
            p = Point()
            p.x = float(wp[0])
            p.y = float(wp[1])
            p.z = 0.04
            marker.points.append(p)

        self.marker_pub.publish(marker)

    def publish_robot(self):
        marker = Marker()

        marker.header.frame_id = 'odom'
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = 'robot'
        marker.id = 2
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        marker.scale.x = 0.30
        marker.scale.y = 0.10
        marker.scale.z = 0.10

        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0

        marker.pose.position.x = float(self.pose[0])
        marker.pose.position.y = float(self.pose[1])
        marker.pose.position.z = 0.05

        qz, qw = self.yaw_to_quaternion(self.pose[2])

        marker.pose.orientation.z = qz
        marker.pose.orientation.w = qw

        self.marker_pub.publish(marker)

    def update_path(self):
        pose = PoseStamped()

        pose.header.frame_id = 'odom'
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = float(self.pose[0])
        pose.pose.position.y = float(self.pose[1])
        pose.pose.position.z = 0.04

        self.path.poses.append(pose)

        if len(self.path.poses) > 1000:
            self.path.poses.pop(0)

        self.path.header.stamp = pose.header.stamp

        self.path_pub.publish(self.path)

    def odom_cb(self, msg):
        p = msg.pose.pose

        self.pose = (
            float(p.position.x),
            float(p.position.y),
            yaw_from_quat(p.orientation)
        )

    def loop(self):
        if self.pose is None:
            return

        self.publish_arena()
        self.publish_safe_zone()
        self.publish_goal()
        self.publish_waypoints()
        self.publish_robot()
        self.update_path()

        if self.state == State.DONE:
            self.publish(0.0, 0.0)
            return

        x, y, yaw = self.pose

        tx, ty, tyaw = self.wps[self.idx]

        dist = math.hypot(
            tx - x,
            ty - y
        )

        heading_err = wrap(
            math.atan2(
                ty - y,
                tx - x
            ) - yaw
        )

        v_t = 0.0
        w_t = 0.0

        if (
            self.state == State.ROTATE
            and dist < self.pos_tol
        ):
            self.set_state(State.ALIGN)

        if self.state == State.ROTATE:

            w_t = clamp(
                self.k_ang * heading_err,
                -self.w_max,
                self.w_max
            )

            if abs(heading_err) < self.align_tol:
                self.set_state(State.DRIVE)

        elif self.state == State.DRIVE:

            if dist < self.pos_tol:
                self.set_state(State.ALIGN)

            elif abs(heading_err) > self.realign:
                self.set_state(State.ROTATE)

            else:
                v_t = clamp(
                    self.k_lin * dist,
                    self.v_min,
                    self.v_max
                )

                v_t *= max(
                    0.0,
                    math.cos(heading_err)
                )

                w_t = clamp(
                    self.k_ang * heading_err,
                    -self.w_max,
                    self.w_max
                )

        elif self.state == State.ALIGN:

            yaw_err = wrap(
                tyaw - yaw
            )

            w_t = clamp(
                self.k_ang * yaw_err,
                -self.w_max,
                self.w_max
            )

            if (
                abs(w_t) < 0.08
                and abs(yaw_err) > self.yaw_tol
            ):
                w_t = math.copysign(
                    0.08,
                    yaw_err
                )

            if abs(yaw_err) < self.yaw_tol:
                self.next_waypoint()

        self.publish(v_t, w_t)

    def publish(self, v_t, w_t):
        self.cmd_v = slew(
            v_t,
            self.cmd_v,
            self.a_lin,
            self.dt
        )

        self.cmd_w = slew(
            w_t,
            self.cmd_w,
            self.a_ang,
            self.dt
        )

        msg = Twist()

        msg.linear.x = self.cmd_v
        msg.angular.z = self.cmd_w

        self.pub.publish(msg)

    def set_state(self, state):
        if state != self.state:
            self.state = state

    def next_waypoint(self):
        self.idx += 1

        if self.idx >= len(self.wps):
            self.state = State.DONE
            self.get_logger().info(
                'All waypoints complete. Stopping.'
            )
        else:
            self.state = State.ROTATE


def main(args=None):
    rclpy.init(args=args)

    node = WaypointNavigator()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.pub.publish(Twist())
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()