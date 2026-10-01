from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_map_file = PathJoinSubstitution(
        [
            FindPackageShare("occupancygrid_creator"),
            "config",
            "map_hovergames_hmpc.yaml",
        ]
    )

    map_file_argument = DeclareLaunchArgument(
        "map_file",
        default_value=default_map_file,
        description="Occupancy-grid configuration file",
    )

    occupancygrid_node = Node(
        package="occupancygrid_creator",
        executable="occupancygrid_creator_node",
        name="occupancygrid_creator_node",
        output="screen",
        parameters=[LaunchConfiguration("map_file")],
    )

    return LaunchDescription(
        [
            map_file_argument,
            occupancygrid_node,
        ]
    )