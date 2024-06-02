from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution

from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="joy_teleop",
            executable="joy_teleop",
            name="joy_teleop",
            parameters=[{
                "x_axis": "1",
                "y_axis": "0",
                "z_axis": "2",
                "th_axis": "5",
                "lidar_btn": "7"
            }],
        )
    ])