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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetParameter, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_name = LaunchConfiguration("robot_name")
    camera_name = LaunchConfiguration("camera_name")

    launch_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [FindPackageShare("realsense2_camera"), "launch", "rs_launch.py"]
                )
            ]
        ),
        launch_arguments={
            "camera_name": camera_name,
            "initial_reset": "True",
            "rgb_camera.profile": "640x360x30",
            "depth_module.profile": "640x360x30",
            "enable_depth": "True",
            "enable_color": "True",
            "rgb.enable_auto_white_balance": "False",
            "rgb_enable_white_balance": "3800.0",
            "rgb_enable_auto_exposure": "False",
            "enable_infra1": "False",
            "enable_infra2": "False",
            "enable_accel":"True",
            "enable_gyro": "True",
            "accel_fps": "200",
            "gyro_fps":"200",
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_name", default_value="robot"),
            DeclareLaunchArgument("camera_name", default_value="camera"),
            GroupAction(
                actions=[
                    PushRosNamespace(robot_name),
                    launch_camera,
                ]
            ),
        ]
    )
