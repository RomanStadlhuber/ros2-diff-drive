import os
from os import environ
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    env={"IGN_GAZEBO_SYSTEM_PLUGIN_PATH": environ["LD_LIBRARY_PATH"]}

    return LaunchDescription([
        ExecuteProcess(
            cmd=[
                # NOTE this path ONLY works when launched from /robot_ws/. !!
                "ign gazebo", "../ignition_ws/differential-drive-robot.sdf"
            ],
            output="screen",
            additional_env=env,
            shell=True
        ),
        Node(
            package="wheel_control",
            namespace="diff_drive",
            executable="wheel_driver",
        ),
        Node(
            package="localization",
            namespace="diff_drive",
            executable="odometry"
        ),
    ])