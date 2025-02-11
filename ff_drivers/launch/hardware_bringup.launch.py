# MIT License
#
# Copyright (c) 2023 Stanford Autonomous Systems Lab
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    robot_name = LaunchConfiguration("robot_name")

    return LaunchDescription(
        [
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
                parameters=[
                    {
                        "server": "192.168.1.3",
                        "port": 3883,
                    }
                ],
            ),
            Node(
                package="ff_control",
                executable="safety_filter",
                name="safety_filter",
                namespace=robot_name,
            ),
        ]
    )
