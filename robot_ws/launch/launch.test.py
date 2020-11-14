from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
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