from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="trajectory",
                executable="trajectory_node",
                name="trajectory_node",
                output="screen",
            )
        ]
    )
