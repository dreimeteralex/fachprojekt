from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    config_file = LaunchConfiguration("slam_config")

    return LaunchDescription([
        DeclareLaunchArgument(
            "slam_config",
            default_value=os.path.join(
                get_package_share_directory("fp_slam"),
                "config",
                "slam.yaml"
            ),
            description="Path to the slam_toolbox config file"
        ),

        PushRosNamespace("robot"),

        Node(
            package="slam_toolbox",
            executable="async_slam_toolbox_node",
            name="slam_toolbox",
            output="screen",
            parameters=[config_file],
            remappings=[
                ("/tf", "tf"),
                ("/tf_static", "tf_static"),
            ]
        )
    ])
