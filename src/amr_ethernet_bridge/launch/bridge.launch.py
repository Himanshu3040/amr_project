from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='amr_ethernet_bridge',
            executable='tcp_bridge_node',
            name='tcp_bridge_node',
            parameters=[{
                'host': '0.0.0.0',
                'port': 8888,
            }],
            output='screen',
        ),
    ])
