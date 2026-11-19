#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker


class WorkspaceTest(Node):

    def __init__(self):
        super().__init__("workspace_test")

        self.pub = self.create_publisher(
            Marker,
            "/workspace_marker",
            10
        )

        self.timer = self.create_timer(
            0.5,
            self.publish_marker
        )

    def publish_marker(self):

        marker = Marker()

        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = "workspace_test"
        marker.id = 0

        # Flat rectangle
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        # ------------------------------------------
        # Workspace position
        # ------------------------------------------

        marker.pose.position.x = 0.0
        marker.pose.position.y = 2.00
        marker.pose.position.z = 0.005

        marker.pose.orientation.w = 1.0

        # ------------------------------------------
        # Workspace size
        # ------------------------------------------

        marker.scale.x = 4.0
        marker.scale.y = 4.0
        marker.scale.z = 0.01

        # ------------------------------------------
        # Semi-transparent green
        # ------------------------------------------

        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.35

        self.pub.publish(marker)


def main():

    rclpy.init()

    node = WorkspaceTest()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
