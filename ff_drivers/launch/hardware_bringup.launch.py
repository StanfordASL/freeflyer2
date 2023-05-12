from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
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
            package="ff_drivers",
            executable="thruster_node",
            name="thruster_node",
            namespace=robot_name,
        ),
        Node(
            package="vrpn_mocap",
            executable="client_node",
            name="vrpn_client_node",
            namespace=PathJoinSubstitution([robot_name, "mocap"]),
            # ASL optitrack IP and port
            parameters=[{
                "server": "192.168.1.8",
                "port": 3883,
            }],
        ),
    ])
