import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math

class FakeTF(Node):

    def __init__(self):
        super().__init__('fake_tf')
        self.br = TransformBroadcaster(self)
        self.timer = self.create_timer(0.05, self.broadcast)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

    def broadcast(self):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"

        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0

        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)

        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.br.sendTransform(t)

def main():
    rclpy.init()
    node = FakeTF()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()