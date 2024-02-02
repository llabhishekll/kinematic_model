from pathlib import Path
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # return launch
    return LaunchDescription(
        [
            Node(
                package="kinematic_model",
                executable="kinematic_model_node",
                name="kinematic_model_node",
                output="screen",
            ),
            Node(
                package="eight_trajectory",
                executable="eight_trajectory_node",
                name="eight_trajectory_node",
                output="screen",
            ),
        ]
    )