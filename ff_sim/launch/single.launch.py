from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    robot_name = LaunchConfiguration("robot_name")

    return LaunchDescription([
        DeclareLaunchArgument("robot_name", default_value="robot"),
        Node(
            package="ff_params",
            executable="robot_params_node",
            name="robot_params_node",
            namespace=robot_name,
        ),
        Node(
            package="ff_sim",
            executable="simulator_node",
            name="simulator_node",
            namespace=robot_name,
            parameters=[{
                "robot_name": robot_name,
            }],
        ),
    ])
