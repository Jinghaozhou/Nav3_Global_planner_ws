from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution([
        FindPackageShare("pct_planner"), "config", "pct_planner.yaml"
    ])

    def launch_node(context):
        overrides = {}
        for name in ("map", "map_dir"):
            value = LaunchConfiguration(name).perform(context)
            if value:
                overrides[name] = value
        parameters = [config]
        if overrides:
            parameters.append(overrides)
        return [Node(
            package="pct_planner",
            executable="pct_planner_node",
            name="pct_planner",
            output="screen",
            parameters=parameters,
        )]

    return LaunchDescription([
        DeclareLaunchArgument("map", default_value=""),
        DeclareLaunchArgument("map_dir", default_value=""),
        OpaqueFunction(function=launch_node),
    ])
