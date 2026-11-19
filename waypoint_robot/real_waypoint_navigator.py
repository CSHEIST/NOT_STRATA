#!/usr/bin/env python3

import math
import socket

import rclpy
from rclpy.node import Node


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class RealWaypointNavigator(Node):

    def __init__(self):
        super().__init__('real_waypoint_navigator')

        self.declare_parameter(
            'waypoints',
            [
                0.1005, 0.6740, 180.0,
                -0.1005, 0.6740, 180.0,
                -0.1005, 0.0000, -90.0,
                0.1005, 0.0000, 0.0,
                0.1005, -0.6500, -90.0
            ]
        )

        self.declare_parameter('arduino_ip', '192.168.4.1')
        self.declare_parameter('udp_port', 8888)

        self.declare_parameter('drive_pwm', 120)
        self.declare_parameter('turn_pwm', 120)

        self.declare_parameter('forward_speed', 0.240)
        self.declare_parameter('turn_90_time', 1.40)

        self.declare_parameter('start_delay', 2.0)
        self.declare_parameter('end_waypoint_number', 2)

        flat = list(self.get_parameter('waypoints').value)

        if len(flat) % 3 != 0:
            raise ValueError(
                'waypoints must be [x, y, yaw_deg] triples'
            )

        self.waypoints = []

        for i in range(0, len(flat), 3):
            self.waypoints.append(
                (
                    float(flat[i]),
                    float(flat[i + 1]),
                    math.radians(float(flat[i + 2]))
                )
            )

        self.ip = self.get_parameter('arduino_ip').value
        self.port = int(self.get_parameter('udp_port').value)

        self.drive_pwm = int(
            self.get_parameter('drive_pwm').value
        )

        self.turn_pwm = int(
            self.get_parameter('turn_pwm').value
        )

        self.forward_speed = float(
            self.get_parameter('forward_speed').value
        )

        self.turn_90_time = float(
            self.get_parameter('turn_90_time').value
        )

        self.start_delay = float(
            self.get_parameter('start_delay').value
        )

        self.end_waypoint = int(
            self.get_parameter('end_waypoint_number').value
        )

        if self.end_waypoint < 2:
            raise ValueError(
                'end_waypoint_number must be at least 2'
            )

        if self.end_waypoint > len(self.waypoints):
            self.end_waypoint = len(self.waypoints)

        self.sock = socket.socket(
            socket.AF_INET,
            socket.SOCK_DGRAM
        )

        self.segment = 1
        self.current_yaw = self.waypoints[0][2]

        self.state = 'START_DELAY'
        self.state_start = self.now()

        self.turn_time = 0.0
        self.turn_left = False
        self.drive_time = 0.0

        self.prepare_segment()

        self.timer = self.create_timer(
            0.05,
            self.control_loop
        )

        self.get_logger().info(
            'REAL waypoint navigator started'
        )

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def elapsed(self):
        return self.now() - self.state_start

    def send(self, left, right):
        packet = f'V,{int(left)},{int(right)}'
        self.sock.sendto(
            packet.encode(),
            (self.ip, self.port)
        )

    def stop(self):
        self.sock.sendto(
            b'STOP',
            (self.ip, self.port)
        )

    def prepare_segment(self):

        if self.segment >= self.end_waypoint:
            self.finish()
            return

        start = self.waypoints[self.segment - 1]
        target = self.waypoints[self.segment]

        dx = target[0] - start[0]
        dy = target[1] - start[1]

        distance = math.hypot(dx, dy)

        target_yaw = target[2]

        yaw_change = wrap_angle(
            target_yaw - self.current_yaw
        )

        angle_deg = math.degrees(yaw_change)

        self.turn_time = (
            abs(angle_deg) / 90.0
        ) * self.turn_90_time

        self.turn_left = yaw_change > 0.0

        self.drive_time = (
            distance / self.forward_speed
        )

        self.get_logger().info(
            f'WP{self.segment} -> WP{self.segment + 1}: '
            f'distance={distance:.3f} m, '
            f'turn={angle_deg:.1f} deg, '
            f'turn_time={self.turn_time:.2f} s, '
            f'drive_time={self.drive_time:.2f} s'
        )

        if self.turn_time > 0.05:
            self.state = 'TURN'
        else:
            self.state = 'DRIVE'

        self.state_start = self.now()

    def control_loop(self):

        if self.state == 'START_DELAY':

            self.stop()

            if self.elapsed() >= self.start_delay:
                self.prepare_segment()

            return

        if self.state == 'TURN':

            if self.turn_left:
                self.send(
                    -self.turn_pwm,
                    self.turn_pwm
                )
            else:
                self.send(
                    self.turn_pwm,
                    -self.turn_pwm
                )

            if self.elapsed() >= self.turn_time:
                self.stop()

                self.current_yaw = self.waypoints[
                    self.segment
                ][2]

                self.state = 'DRIVE'
                self.state_start = self.now()

            return

        if self.state == 'DRIVE':

            self.send(
                self.drive_pwm,
                self.drive_pwm
            )

            if self.elapsed() >= self.drive_time:
                self.stop()

                self.get_logger().info(
                    f'Reached approximate WP{self.segment + 1}'
                )

                if self.segment + 1 >= self.end_waypoint:
                    self.finish()
                else:
                    self.segment += 1
                    self.prepare_segment()

            return

        if self.state == 'DONE':

            self.stop()

    def finish(self):

        self.stop()

        self.state = 'DONE'

        self.get_logger().info(
            'Waypoint test complete. Robot stopped.'
        )

    def destroy_node(self):

        try:
            self.stop()
        except Exception:
            pass

        self.sock.close()
        super().destroy_node()


def main(args=None):

    rclpy.init(args=args)

    node = RealWaypointNavigator()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.stop()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
