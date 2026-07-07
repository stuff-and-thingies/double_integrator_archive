from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="fake_drone_sim",
                executable="fake_drone_sim_node",
                name="fake_drone_sim_node",
                output="screen",
            )
        ]
    )
