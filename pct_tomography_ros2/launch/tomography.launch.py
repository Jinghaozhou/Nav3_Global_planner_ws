from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("scene", default_value="building"),
        Node(
            package="pct_tomography_ros2",
            executable="tomography_node",
            name="pct_tomography",
            output="screen",
            parameters=[{
                "scene": LaunchConfiguration("scene"),
            }],
        ),
    ])
