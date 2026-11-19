#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

BASE_REAL_MIN = 10.0
BASE_REAL_MAX = 170.0

SHOULDER_REAL_MIN = 17.0
SHOULDER_REAL_MAX = 172.0

URDF_MIN = -90.0
URDF_MAX = 90.0


def map_angle(urdf_angle, real_min, real_max):
    urdf_angle = max(URDF_MIN, min(URDF_MAX, urdf_angle))

    ratio = (urdf_angle - URDF_MIN) / (URDF_MAX - URDF_MIN)

    return real_min + ratio * (real_max - real_min)


class ArmCalibration(Node):

    def __init__(self):
        super().__init__("arm_calibration")

        self.subscription = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_callback,
            10
        )

    def joint_callback(self, msg):

        try:
            base_index = msg.name.index("base_joint")
            shoulder_index = msg.name.index("shoulder_joint")
        except ValueError:
            return

        base_urdf = msg.position[base_index] * 180.0 / 3.14159265359
        shoulder_urdf = msg.position[shoulder_index] * 180.0 / 3.14159265359

        base_real = map_angle(
            base_urdf,
            BASE_REAL_MIN,
            BASE_REAL_MAX
        )

        shoulder_real = map_angle(
            shoulder_urdf,
            SHOULDER_REAL_MIN,
            SHOULDER_REAL_MAX
        )

        print(
            f"\r"
            f"URDF Base={base_urdf:7.2f}°, "
            f"Shoulder={shoulder_urdf:7.2f}° | "
            f"REAL Base={base_real:7.2f}°, "
            f"Shoulder={shoulder_real:7.2f}°",
            end="",
            flush=True
        )


def main():

    rclpy.init()

    node = ArmCalibration()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
