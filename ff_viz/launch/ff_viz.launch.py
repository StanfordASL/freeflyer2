import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    urdf_file_name = "ff.urdf"
    urdf = os.path.join(get_package_share_directory("ff_viz"), "model", urdf_file_name)
    with open(urdf, "r") as f:
        robot_desc = f.read()

    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[{"robot_description": robot_desc}],
        ),
        Node(
            package="ff_viz",
            executable="renderer_node",
            name="renderer_node",
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
