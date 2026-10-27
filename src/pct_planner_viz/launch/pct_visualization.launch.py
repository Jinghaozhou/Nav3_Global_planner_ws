from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution([
        FindPackageShare("pct_planner_viz"), "rviz", "pct_visualization.rviz"
    ])
    return LaunchDescription([
        DeclareLaunchArgument("planner_node", default_value="/pct_planner"),
        DeclareLaunchArgument("max_points", default_value="200000"),
        DeclareLaunchArgument("rviz", default_value="true"),
        Node(
            package="pct_planner_viz",
            executable="pct_map_visualizer",
            name="pct_map_visualizer",
            output="screen",
            parameters=[{
                "planner_node": LaunchConfiguration("planner_node"),
                "max_points": LaunchConfiguration("max_points"),
            }],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="pct_rviz",
            arguments=["-d", config],
            output="screen",
            condition=IfCondition(LaunchConfiguration("rviz")),
        ),
    ])
