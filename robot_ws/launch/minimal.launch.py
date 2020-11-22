import os
import pathlib
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    param_file_path = str(pathlib.Path(__file__).parents[0]) + "/common-args.yaml"

    print("launching with arguments from " + param_file_path)

    return LaunchDescription([
        Node(
            package="wheel_control",
            namespace="diff_drive",
            executable="wheel_driver",
            parameters=[
                param_file_path
            ]
        ),
        Node(
            package="localization",
            namespace="diff_drive",
            executable="odometry",
            parameters=[
                param_file_path
            ]
        )
    ])