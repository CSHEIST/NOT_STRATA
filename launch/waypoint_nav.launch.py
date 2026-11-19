import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_share = get_package_share_directory('waypoint_robot')

    cfg = os.path.join(pkg_share, 'config', 'waypoints.yaml')

    # ✅ FIXED PATHS (IMPORTANT)
    robot_file = os.path.join(pkg_share, 'urdf', 'robot.urdf')
    arena_file = os.path.join(pkg_share, 'urdf', 'arena.urdf')

    # ✅ READ URDF FILES
    with open(robot_file, 'r') as f:
        robot_desc = f.read()

    with open(arena_file, 'r') as f:
        arena_desc = f.read()

    return LaunchDescription([

        # ---------------- ARG ----------------
        DeclareLaunchArgument('sim', default_value='true'),

        # ---------------- SIM ROBOT ----------------
        Node(
            package='waypoint_robot',
            executable='sim_robot',
            name='sim_robot',
            parameters=[cfg],
            output='screen',
            condition=IfCondition(LaunchConfiguration('sim'))
        ),

        # ---------------- NAVIGATOR ----------------
        Node(
            package='waypoint_robot',
            executable='waypoint_navigator',
            name='waypoint_navigator',
            parameters=[cfg],
            output='screen'
        ),

        # ❌ DO NOT USE tf_broadcaster (conflicts)

        # ---------------- ROBOT MODEL ----------------
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': robot_desc
            }],
            output='screen'
        ),

        # ---------------- ARENA MODEL ----------------
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='arena_publisher',
            parameters=[{
                'robot_description': arena_desc
            }],
            output='screen'
        ),

        # 🔥 CRITICAL FIX: CONNECT ARENA TO TF TREE
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='arena_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'floor'],
        ),

    ])