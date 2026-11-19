#!/usr/bin/env python3

import math
import csv
import os
import numpy as np


# ============================================================
# Rotation matrices
# ============================================================

def Rx(a):
    c = math.cos(a)
    s = math.sin(a)

    return np.array([
        [1, 0, 0],
        [0, c, -s],
        [0, s, c]
    ], dtype=float)


def Ry(a):
    c = math.cos(a)
    s = math.sin(a)

    return np.array([
        [c, 0, s],
        [0, 1, 0],
        [-s, 0, c]
    ], dtype=float)


def Rz(a):
    c = math.cos(a)
    s = math.sin(a)

    return np.array([
        [c, -s, 0],
        [s, c, 0],
        [0, 0, 1]
    ], dtype=float)


def RPY(roll, pitch, yaw):
    # URDF RPY convention
    return Rz(yaw) @ Ry(pitch) @ Rx(roll)


# ============================================================
# Homogeneous transformation
# ============================================================

def make_transform(R, xyz):

    T = np.eye(4)

    T[:3, :3] = R
    T[:3, 3] = xyz

    return T


# ============================================================
# Forward Kinematics
#
# These values come directly from your current URDF/Xacro:
#
# base_joint:
#   origin = 0 0 0.57
#
# shoulder_joint:
#   origin = 0 0.095 0.4
#   axis   = 1 0 0
#
# elbow_joint:
#   origin = 0 0 1.2
#   axis   = 1 0 0
#
# wrist_joint:
#   origin = 0 0.912 0.07
#   axis   = 0 1 0
#
# gripper_joint:
#   origin = -0.07 0.12 -0.1
#   rpy    = -1.5708 0 0
#
# tool0:
#   origin = -0.04 0.07 1.35
#
# gripper_joint is fixed at 0 for this workspace scan.
# ============================================================

def forward_kinematics(base,
                        shoulder,
                        elbow,
                        wrist):

    T = np.eye(4)

    # --------------------------------------------------------
    # Base
    # --------------------------------------------------------

    T = T @ make_transform(
        np.eye(3),
        np.array([0.0, 0.0, 0.57])
    )

    T = T @ make_transform(
        Rz(base),
        np.array([0.0, 0.0, 0.0])
    )

    # --------------------------------------------------------
    # Shoulder
    # --------------------------------------------------------

    T = T @ make_transform(
        np.eye(3),
        np.array([0.0, 0.095, 0.4])
    )

    T = T @ make_transform(
        Rx(shoulder),
        np.array([0.0, 0.0, 0.0])
    )

    # --------------------------------------------------------
    # Elbow
    # --------------------------------------------------------

    T = T @ make_transform(
        np.eye(3),
        np.array([0.0, 0.0, 1.2])
    )

    T = T @ make_transform(
        Rx(elbow),
        np.array([0.0, 0.0, 0.0])
    )

    # --------------------------------------------------------
    # Wrist
    # --------------------------------------------------------

    T = T @ make_transform(
        np.eye(3),
        np.array([0.0, 0.912, 0.07])
    )

    T = T @ make_transform(
        Ry(wrist),
        np.array([0.0, 0.0, 0.0])
    )

    # --------------------------------------------------------
    # Gripper joint
    # Fixed at 0 rad for workspace calculation
    # --------------------------------------------------------

    T = T @ make_transform(
        RPY(-1.5708, 0.0, 0.0),
        np.array([-0.07, 0.12, -0.1])
    )

    # --------------------------------------------------------
    # tool0
    # --------------------------------------------------------

    T = T @ make_transform(
        np.eye(3),
        np.array([-0.04, 0.07, 1.35])
    )

    return T


# ============================================================
# Check zero pose against TF
# ============================================================

zero_T = forward_kinematics(
    0.0,
    0.0,
    0.0,
    0.0
)

zero_position = zero_T[:3, 3]

print()
print("==============================================")
print("        ROBOT ARM WORKSPACE SCANNER")
print("==============================================")

print()
print("Zero-pose tool0 position from FK:")

print(
    f"X = {zero_position[0]:.3f} m"
)

print(
    f"Y = {zero_position[1]:.3f} m"
)

print(
    f"Z = {zero_position[2]:.3f} m"
)

print()
print("Expected TF reference:")
print("X = -0.110 m")
print("Y =  2.477 m")
print("Z =  2.070 m")

print()


# ============================================================
# Joint sampling
# ============================================================

BASE_SAMPLES = 72
OTHER_SAMPLES = 21

base_values = np.linspace(
    -math.pi,
    math.pi,
    BASE_SAMPLES,
    endpoint=False
)

shoulder_values = np.linspace(
    -math.pi / 2,
    math.pi / 2,
    OTHER_SAMPLES
)

elbow_values = np.linspace(
    -math.pi / 2,
    math.pi / 2,
    OTHER_SAMPLES
)

wrist_values = np.linspace(
    -math.pi / 2,
    math.pi / 2,
    OTHER_SAMPLES
)


# ============================================================
# Scan workspace
# ============================================================

points = []

total = (
    len(base_values)
    * len(shoulder_values)
    * len(elbow_values)
    * len(wrist_values)
)

count = 0

print(f"Total configurations: {total}")
print()
print("Scanning workspace...")
print()


for base in base_values:

    for shoulder in shoulder_values:

        for elbow in elbow_values:

            for wrist in wrist_values:

                T = forward_kinematics(
                    base,
                    shoulder,
                    elbow,
                    wrist
                )

                p = T[:3, 3]

                points.append([
                    p[0],
                    p[1],
                    p[2]
                ])

                count += 1

    percent = 100.0 * count / total

    print(
        f"\rProgress: {percent:6.2f}%",
        end=""
    )


print()
print()

points = np.array(points)


# ============================================================
# Workspace limits
# ============================================================

xmin, ymin, zmin = np.min(points, axis=0)
xmax, ymax, zmax = np.max(points, axis=0)


print("==============================================")
print("           REACHABLE WORKSPACE")
print("==============================================")

print()

print(
    f"X range : {xmin:.3f} m  to  {xmax:.3f} m"
)

print(
    f"Y range : {ymin:.3f} m  to  {ymax:.3f} m"
)

print(
    f"Z range : {zmin:.3f} m  to  {zmax:.3f} m"
)

print()

print(
    f"X range : {xmin*1000:.1f} mm  to  {xmax*1000:.1f} mm"
)

print(
    f"Y range : {ymin*1000:.1f} mm  to  {ymax*1000:.1f} mm"
)

print(
    f"Z range : {zmin*1000:.1f} mm  to  {zmax*1000:.1f} mm"
)

print()


# ============================================================
# Save CSV
# ============================================================

output_file = os.path.expanduser(
    "~/robotic_arm_ws/workspace_points.csv"
)

with open(
    output_file,
    "w",
    newline=""
) as f:

    writer = csv.writer(f)

    writer.writerow([
        "x_m",
        "y_m",
        "z_m"
    ])

    writer.writerows(points)


print("Workspace saved to:")
print(output_file)

print()
print("==============================================")
print("                SCAN COMPLETE")
print("==============================================")
