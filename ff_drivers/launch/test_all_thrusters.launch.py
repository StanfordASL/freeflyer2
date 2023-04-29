from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    robot_name = LaunchConfiguration("robot_name")

    return LaunchDescription([
        DeclareLaunchArgument("robot_name", default_value="robot"),
        Node(
            package="ff_drivers",
            executable="thruster_node",
            name="thruster_node",
            namespace=robot_name,
        ),
        Node(
            package="ff_drivers",
            executable="test_all_thrusters",
            name="test_all_thrusters",
            namespace=robot_name,
        )
    ])
