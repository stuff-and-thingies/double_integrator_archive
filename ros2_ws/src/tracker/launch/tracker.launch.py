from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="tracker",
                executable="tracker_node",
                name="tracker_node",
                output="screen",
            )
        ]
    )
