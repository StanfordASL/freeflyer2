import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue

def generate_launch_description():
    urdf = os.path.join(get_package_share_directory("ff_viz"), "model", "ff.urdf.xacro")
    robot_name = LaunchConfiguration("robot_name")

    return LaunchDescription([
        DeclareLaunchArgument("robot_name", default_value="robot"),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            namespace=robot_name,
            parameters=[{
                "robot_description": ParameterValue(
                    Command(["xacro ", urdf, " robot_name:=", robot_name]),
                    value_type=str
                )
            }],
        ),
        Node(
            package="ff_viz",
            executable="renderer_node",
            name="renderer_node",
            parameters=[{"robot_names": [robot_name]}]
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=[
                "-d",
                os.path.join(get_package_share_directory("ff_viz"), "rviz", "default.rviz"),
            ],
        )
    ])
