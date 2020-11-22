import os
import pathlib
from os import environ
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    param_file_path = str(pathlib.Path(__file__).parents[0]) + "/common-args.yaml"

    print("launching with arguments from " + param_file_path)

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
        ),
        # start the bridge to ign gazebo
        ExecuteProcess(
            cmd=[
                "ros2 run ros_ign_bridge parameter_bridge",
                "/diff_drive/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist"
            ],
            shell=True,
            additional_env=env,
        )
    ])