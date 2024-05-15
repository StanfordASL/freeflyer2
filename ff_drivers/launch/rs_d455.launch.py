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
                package="realsense2_camera",
                executable="realsense2_camera_node",
                name="realsense2_camera_node",
                namespace=PathJoinSubstitution([robot_name, "camera"]),
                parameters=[
                    {
                        "depth_module.profile": "640x360x30",
                        "rgb_camera.profile": "640x360x30",
                        "rgb_camera.enable_auto_exposure": False,
                        "rgb_camera.enable_auto_white_balance": False,
                        "rgb_camera.white_balance": 3800.0,
                        "enable_infra1": False,
                        "enable_infra2": False,
                    }
                ],
            ),
        ]
    )
