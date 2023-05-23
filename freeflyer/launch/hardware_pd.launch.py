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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_name = LaunchConfiguration("robot_name")
    rviz = LaunchConfiguration("rviz")

    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_name", default_value="robot"),
            DeclareLaunchArgument("rviz", default_value="false",
                                  description="set to true to launch rviz",
                                  choices=["true", "false"]),
            IncludeLaunchDescription(
               PathJoinSubstitution([
                   FindPackageShare("ff_viz"),
                   "launch",
                   "ff_viz.launch.py",
               ]),
               launch_arguments={"robot_name": robot_name}.items(),
               condition=IfCondition(rviz),
            ),
            Node(
                package="ff_control",
                executable="pd_ctrl_node",
                name="pd_ctrl_node",
                namespace=robot_name,
            ),
            Node(
                package="ff_estimate",
                executable="mocap_estimator_node",
                name="mocap_estimator_node",
                namespace=robot_name,
                parameters=[
                    {
                        "pose_channel": PathJoinSubstitution(
                            [
                                "mocap",
                                robot_name,
                                "pose",
                            ]
                        ),
                    }
                ],
            ),
        ]
    )
