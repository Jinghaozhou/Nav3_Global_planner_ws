from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="pct_planner_ros2",
            executable="planner_node",
            name="pct_planner",
            output="screen",
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare("pct_planner_ros2"),
                    "config",
                    "planner.yaml",
                ])
            ],
        ),
    ])
