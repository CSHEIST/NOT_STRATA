
#!/usr/bin/env python3

import math
import random
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from python_qt_binding.QtCore import Qt, QTimer
from python_qt_binding.QtWidgets import (
    QApplication,
    QMainWindow,
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSlider,
    QSpinBox,
    QGroupBox,
)


# Joint limits in degrees.
# Keep these consistent with the limits in your URDF.
JOINT_LIMITS = {
    "base_joint": (-73, 73),
    "shoulder_joint": (-90, 90),
    "elbow_joint": (-90, 90),
    "wrist_joint": (-90, 90),
}


class ArmControlGUI(QMainWindow):

    def __init__(self, node):
        super().__init__()

        self.node = node
        self.sliders = {}
        self.value_boxes = {}

        self.joint_pub = node.create_publisher(
            JointState, "/joint_states", 10
        )

        self.gripper_pub = node.create_publisher(
            String, "/gripper_command", 10
        )

        self.setWindowTitle("Robotic Arm Controller")
        self.resize(390, 570)

        central = QWidget()
        self.setCentralWidget(central)

        layout = QVBoxLayout(central)

        # Randomize and Center buttons
        top_row = QHBoxLayout()

        random_button = QPushButton("Randomize")
        random_button.clicked.connect(self.randomize_joints)

        center_button = QPushButton("Center")
        center_button.clicked.connect(self.center_joints)

        top_row.addWidget(random_button)
        top_row.addWidget(center_button)
        layout.addLayout(top_row)

        # Joint sliders
        joint_group = QGroupBox("Joint Control")
        joint_layout = QVBoxLayout(joint_group)

        for joint_name, limits in JOINT_LIMITS.items():
            minimum, maximum = limits

            row = QVBoxLayout()

            heading = QHBoxLayout()

            name_label = QLabel(joint_name)
            name_label.setStyleSheet("font-weight: bold;")

            value_box = QSpinBox()
            value_box.setRange(minimum, maximum)
            value_box.setValue(0)
            value_box.setSuffix("°")
            value_box.setFixedWidth(90)

            heading.addWidget(name_label)
            heading.addStretch()
            heading.addWidget(value_box)

            slider = QSlider(Qt.Horizontal)
            slider.setRange(minimum, maximum)
            slider.setValue(0)

            slider.valueChanged.connect(
                lambda value, box=value_box: box.setValue(value)
            )

            value_box.valueChanged.connect(
                lambda value, s=slider: s.setValue(value)
            )

            self.sliders[joint_name] = slider
            self.value_boxes[joint_name] = value_box

            row.addLayout(heading)
            row.addWidget(slider)

            joint_layout.addLayout(row)

        layout.addWidget(joint_group)

        # Separate gripper buttons — no gripper slider
        gripper_group = QGroupBox("Gripper Control")
        gripper_layout = QVBoxLayout(gripper_group)

        button_row = QHBoxLayout()

        self.open_button = QPushButton("OPEN")
        self.close_button = QPushButton("CLOSE")

        self.open_button.setMinimumHeight(48)
        self.close_button.setMinimumHeight(48)

        self.open_button.setStyleSheet(
            "QPushButton { background-color: #218838;"
            " color: white; font-weight: bold; font-size: 15px; }"
        )

        self.close_button.setStyleSheet(
            "QPushButton { background-color: #c82333;"
            " color: white; font-weight: bold; font-size: 15px; }"
        )

        self.open_button.clicked.connect(
            lambda: self.send_gripper_command("OPEN")
        )

        self.close_button.clicked.connect(
            lambda: self.send_gripper_command("CLOSE")
        )

        button_row.addWidget(self.open_button)
        button_row.addWidget(self.close_button)

        gripper_layout.addLayout(button_row)

        self.gripper_status = QLabel("Gripper: No command sent")
        self.gripper_status.setAlignment(Qt.AlignCenter)
        gripper_layout.addWidget(self.gripper_status)

        layout.addWidget(gripper_group)

        # Publish joint states periodically.
        self.publish_timer = QTimer(self)
        self.publish_timer.timeout.connect(self.publish_joint_states)
        self.publish_timer.start(50)

        # Process ROS events without blocking the Qt GUI.
        self.ros_timer = QTimer(self)
        self.ros_timer.timeout.connect(
            lambda: rclpy.spin_once(node, timeout_sec=0.0)
            if rclpy.ok() else None
        )
        self.ros_timer.start(10)

        self.publish_joint_states()

    def publish_joint_states(self):
        if not rclpy.ok():
            return

        msg = JointState()
        msg.header.stamp = self.node.get_clock().now().to_msg()

        for joint_name in JOINT_LIMITS:
            angle_deg = self.sliders[joint_name].value()

            msg.name.append(joint_name)
            msg.position.append(math.radians(angle_deg))

        self.joint_pub.publish(msg)

    def send_gripper_command(self, command):
        msg = String()
        msg.data = command

        self.gripper_pub.publish(msg)

        self.gripper_status.setText(
            f"Gripper command sent: {command}"
        )

        self.node.get_logger().info(
            f"Published gripper command: {command}"
        )

    def center_joints(self):
        for slider in self.sliders.values():
            slider.setValue(0)

    def randomize_joints(self):
        for joint_name, slider in self.sliders.items():
            minimum, maximum = JOINT_LIMITS[joint_name]
            slider.setValue(random.randint(minimum, maximum))


def main(args=None):
    rclpy.init(args=args)

    app = QApplication(sys.argv)
    node = Node("arm_control_gui")

    window = ArmControlGUI(node)
    window.show()

    try:
        result = app.exec() if hasattr(app, "exec") else app.exec_()
    finally:
        window.publish_timer.stop()
        window.ros_timer.stop()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

    return result


if __name__ == "__main__":
    sys.exit(main())
