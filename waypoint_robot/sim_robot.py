#!/usr/bin/env python3
"""Lightweight unicycle simulator: /cmd_vel -> integrate pose -> /odom + TF(odom -> base_link)."""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


class SimRobot(Node):
    def __init__(self):
        super().__init__('sim_robot')
        self.declare_parameter('start_x', 0.0)
        self.declare_parameter('start_y', 0.0)
        self.declare_parameter('start_yaw_deg', 0.0)
        self.declare_parameter('rate', 50.0)

        self.x = self.get_parameter('start_x').value
        self.y = self.get_parameter('start_y').value
        self.yaw = math.radians(self.get_parameter('start_yaw_deg').value)
        self.dt = 1.0 / self.get_parameter('rate').value
        self.v = 0.0
        self.w = 0.0

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_br = TransformBroadcaster(self)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 10)
        self.create_timer(self.dt, self.step)
        self.get_logger().info('Sim robot running (unicycle model).')

    def cmd_cb(self, msg):
        self.v = msg.linear.x
        self.w = msg.angular.z

    def step(self):
        # Unicycle kinematics
        self.x += self.v * math.cos(self.yaw) * self.dt
        self.y += self.v * math.sin(self.yaw) * self.dt
        self.yaw = math.atan2(math.sin(self.yaw + self.w * self.dt),
                              math.cos(self.yaw + self.w * self.dt))

        now = self.get_clock().now().to_msg()
        qz, qw = math.sin(self.yaw / 2.0), math.cos(self.yaw / 2.0)

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = self.v
        odom.twist.twist.angular.z = self.w
        self.odom_pub.publish(odom)

        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_br.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = SimRobot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
