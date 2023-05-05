from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="vrpn_mocap",
            executable="client_node",
            name="vrpn_client_node",
            namespace="mocap",
            # ASL optitrack IP and port
            parameters=[{
                "server": "192.168.1.8",
                "port": 3883,
            }],
        ),
    ])
