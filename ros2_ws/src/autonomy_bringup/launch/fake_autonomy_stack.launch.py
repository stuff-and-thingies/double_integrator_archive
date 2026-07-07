import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    foxglove_package = get_package_share_directory("foxglove_bridge")

    foxglove_bridge = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(foxglove_package, "launch", "foxglove_bridge_launch.xml")
        ),
        # Optional: Override default arguments like port (defaults to 8765)
        launch_arguments={"port": "8765"}.items(),
    )

    fake_drone_sim_node = Node(
        package="fake_drone_sim",
        executable="fake_drone_sim_node",
        name="fake_drone_sim_node",
        output="screen",
    )

    planner_node = Node(
        package="planner",
        executable="planner_node",
        name="planner_node",
        output="screen",
    )

    tracker_node = Node(
        package="tracker",
        executable="tracker_node",
        name="tracker_node",
        output="screen",
    )

    trajectory_node = Node(
        package="trajectory",
        executable="trajectory_node",
        name="trajectory_node",
        output="screen",
    )

    safety_monitor_node = Node(
        package="safety_monitor",
        executable="safety_monitor_node",
        name="safety_monitor_node",
        output="screen",
    )

    command_gate_node = Node(
        package="command_gate",
        executable="command_gate_node",
        name="command_gate_node",
        output="screen",
    )

    mission_manager_node = Node(
        package="mission_manager",
        executable="mission_manager_node",
        name="mission_manager_node",
        output="screen",
    )

    return LaunchDescription(
        [
            foxglove_bridge,
            fake_drone_sim_node,
            planner_node,
            tracker_node,
            trajectory_node,
            safety_monitor_node,
            command_gate_node,
            mission_manager_node,
        ]
    )
