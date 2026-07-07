from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="command_gate",
                executable="command_gate_node",
                name="command_gate_node",
                output="screen",
            )
        ]
    )
