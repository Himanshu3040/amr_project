#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    amr_controller_pkg = get_package_share_directory('amr_controller')

    use_sim_time_arg = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="True",
        description="Use simulated time"
    )

    joy_teleop = Node(
        package="joy_teleop",
        executable="joy_teleop",
        parameters=[
            os.path.join(amr_controller_pkg, "config", "joy_teleop.yaml"),
            {"use_sim_time": LaunchConfiguration("use_sim_time")}
        ],
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joystick",
        parameters=[
            os.path.join(amr_controller_pkg, "config", "joy_config.yaml"),
            {"use_sim_time": LaunchConfiguration("use_sim_time")}
        ]
    )

    twist_mux_node = Node(
        package="twist_mux",
        executable="twist_mux",
        name="twist_mux",
        parameters=[
            os.path.join(amr_controller_pkg, "config", "twist_mux_locks.yaml"),
            os.path.join(amr_controller_pkg, "config", "twist_mux_topics.yaml"),
            {"use_sim_time": LaunchConfiguration("use_sim_time")}
        ],
        remappings=[
            ("cmd_vel_out", "cmd_vel_combined")
        ]
    )

    turbo_scaler_node = Node(
        package="amr_controller",
        executable="turbo_scaler",
        name="turbo_scaler",
        parameters=[
            os.path.join(amr_controller_pkg, "config", "twist_mux_key.yaml"),
            {"use_sim_time": LaunchConfiguration("use_sim_time")}
        ]
    )

    twist_relay_node = Node(
        package="amr_controller",
        executable="twist_relay",
        name="twist_relay",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}]
    )

    return LaunchDescription([
        use_sim_time_arg,
        joy_teleop,
        joy_node,
        twist_mux_node,
        turbo_scaler_node, 
        twist_relay_node,
    ])