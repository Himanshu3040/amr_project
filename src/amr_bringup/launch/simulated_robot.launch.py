import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    use_slam_arg = DeclareLaunchArgument(
        "use_slam",
        default_value="false"
    )

    gazebo = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("amr_description"),
            "launch",
            "gazebo.launch.py"
        ),
    )
    
    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("amr_controller"),
            "launch",
            "controller.launch.py"
        ),
        launch_arguments={
            "use_simple_controller": "False",
            "use_python": "False"
        }.items(),
    )
    
    teleop = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("amr_controller"),
            "launch",
            "teleop.launch.py"
        ),
        launch_arguments={
            "use_sim_time": "True"
        }.items()
    )
    
    return LaunchDescription([
        use_slam_arg,
        gazebo,
        controller,
        teleop
    ])