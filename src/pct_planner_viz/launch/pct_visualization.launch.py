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
        DeclareLaunchArgument("display_voxel_size", default_value="0.10"),
        DeclareLaunchArgument("marker_size", default_value="0.8"),
        DeclareLaunchArgument("waypoints", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),
        Node(
            package="pct_planner_viz",
            executable="pct_map_visualizer",
            name="pct_map_visualizer",
            output="screen",
            parameters=[{
                "planner_node": LaunchConfiguration("planner_node"),
                "max_points": LaunchConfiguration("max_points"),
                "display_voxel_size": LaunchConfiguration("display_voxel_size"),
            }],
        ),
        Node(
            package="pct_planner_viz",
            executable="pct_waypoint_markers",
            name="pct_waypoint_markers",
            output="screen",
            parameters=[{"marker_size": LaunchConfiguration("marker_size")}],
            condition=IfCondition(LaunchConfiguration("waypoints")),
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
