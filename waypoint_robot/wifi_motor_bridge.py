#!/usr/bin/env python3

import socket
import math
import rclpy

from rclpy.node import Node
from geometry_msgs.msg import Twist


class WiFiMotorBridge(Node):

    def __init__(self):
        super().__init__('wifi_motor_bridge')

        self.declare_parameter('arduino_ip', '192.168.4.1')
        self.declare_parameter('udp_port', 8888)
        self.declare_parameter('wheel_radius', 0.015)
        self.declare_parameter('track_width', 0.115)
        self.declare_parameter('max_pwm', 180)
        self.declare_parameter('max_linear', 0.20)
        self.declare_parameter('timeout', 0.5)

        self.ip = self.get_parameter('arduino_ip').value
        self.port = int(self.get_parameter('udp_port').value)
        self.radius = float(self.get_parameter('wheel_radius').value)
        self.track = float(self.get_parameter('track_width').value)
        self.max_pwm = int(self.get_parameter('max_pwm').value)
        self.max_linear = float(self.get_parameter('max_linear').value)
        self.timeout = float(self.get_parameter('timeout').value)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.last_cmd_time = self.get_clock().now()

        self.sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.timer = self.create_timer(0.1, self.watchdog)

        self.get_logger().info(
            f'WiFi motor bridge → {self.ip}:{self.port}'
        )

    def send(self, left_pwm, right_pwm):

        left_pwm = max(-self.max_pwm, min(self.max_pwm, int(left_pwm)))
        right_pwm = max(-self.max_pwm, min(self.max_pwm, int(right_pwm)))

        packet = f'V,{left_pwm},{right_pwm}'
        self.sock.sendto(packet.encode(), (self.ip, self.port))

    def cmd_vel_callback(self, msg):

        v = float(msg.linear.x)
        w = float(msg.angular.z)

        left_velocity = v - (w * self.track / 2.0)
        right_velocity = v + (w * self.track / 2.0)

        max_wheel_velocity = self.max_linear

        left_velocity = max(
            -max_wheel_velocity,
            min(max_wheel_velocity, left_velocity)
        )

        right_velocity = max(
            -max_wheel_velocity,
            min(max_wheel_velocity, right_velocity)
        )

        left_pwm = (left_velocity / max_wheel_velocity) * self.max_pwm
        right_pwm = (right_velocity / max_wheel_velocity) * self.max_pwm

        self.send(left_pwm, right_pwm)

        self.last_cmd_time = self.get_clock().now()

    def watchdog(self):

        elapsed = (
            self.get_clock().now() - self.last_cmd_time
        ).nanoseconds / 1e9

        if elapsed > self.timeout:
            self.sock.sendto(
                b'STOP',
                (self.ip, self.port)
            )

    def destroy_node(self):

        try:
            self.sock.sendto(
                b'STOP',
                (self.ip, self.port)
            )
        except Exception:
            pass

        self.sock.close()
        super().destroy_node()


def main(args=None):

    rclpy.init(args=args)

    node = WiFiMotorBridge()

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
