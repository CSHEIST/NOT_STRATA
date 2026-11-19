#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class BaseRotation(Node):

    def __init__(self):
        super().__init__('base_rotation')

        self.publisher = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )

        self.angle = 0.0

        # Update every 0.1 second
        self.timer = self.create_timer(0.1, self.update)

        self.get_logger().info('Base rotation controller started')

    def update(self):

        msg = JointState()

        msg.header.stamp = self.get_clock().now().to_msg()

        msg.name = ['base_joint']

        # Convert degrees to radians
        self.angle += math.radians(2.0)

        # Keep angle within 0 to 360 degrees
        if self.angle >= 2.0 * math.pi:
            self.angle -= 2.0 * math.pi

        msg.position = [self.angle]

        self.publisher.publish(msg)


def main(args=None):

    rclpy.init(args=args)

    node = BaseRotation()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
