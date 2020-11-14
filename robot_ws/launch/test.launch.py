import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    pkg_ros_ign_gazebo = get_package_share_directory("ros_ign_gazebo");

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_ign_gazebo, 'launch', 'ign_gazebo.launch.py'),
        ),
    )

    return LaunchDescription([
        Node(
            package="wheel_control",
            namespace="diff_drive",
            executable="wheel_driver",
        ),
        Node(
            package="localization",
            namespace="diff_drive",
            executable="odometry"
        )
    ])