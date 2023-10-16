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
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    urdf = PathJoinSubstitution([FindPackageShare("ff_viz"), "model", "ff.urdf.xacro"])
    default_rviz_config = PathJoinSubstitution([FindPackageShare("ff_viz"), "rviz", "default.rviz"])
    robot_name = LaunchConfiguration("robot_name")
    rviz_config = LaunchConfiguration("rviz_config")

    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_name", default_value="robot"),
            DeclareLaunchArgument("rviz_config", default_value=default_rviz_config),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                namespace=robot_name,
                parameters=[
                    {
                        "robot_description": ParameterValue(
                            Command(["xacro ", urdf, " robot_name:=", robot_name]), value_type=str
                        )
                    }
                ],
            ),
            Node(
                package="ff_viz",
                executable="renderer_node",
                name="renderer_node",
                parameters=[{"robot_names": [robot_name]}],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                namespace=robot_name,
                arguments=[
                    "-d",
                    rviz_config,
                ],
            ),
        ]
    )
