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


import time
import unittest

from ff_msgs.msg import FreeFlyerStateStamped

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_testing.actions import ReadyToTest

import rclpy


ROBOT_NAME = "freeflyer"


def generate_test_description():
    impl = LaunchConfiguration("impl")

    sim_launch = IncludeLaunchDescription(
        PathJoinSubstitution(
            [
                FindPackageShare("ff_sim"),
                "launch",
                "single.launch.py",
            ]
        ),
        launch_arguments={"robot_name": ROBOT_NAME}.items(),
    )
    pd_ctrl_node = Node(
        package="ff_control",
        executable=["pd_ctrl_", impl, "_node"],
        name="pd_ctrl_node",
        namespace=ROBOT_NAME,
    )
    pwm_ctrl_node = Node(
        package="ff_control",
        executable=["pwm_ctrl_cpp_node"],
        name="pwm_ctrl_node",
        namespace=ROBOT_NAME,
        condition=IfCondition(PythonExpression(["'", impl, "'", " == 'py'"])),
    )
    estimator = Node(
        package="ff_estimate",
        executable="moving_avg_estimator_node",
        name="moving_avg_estimator_node",
        namespace=ROBOT_NAME,
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("impl", default_value="cpp", choices=["cpp", "py"]),
            sim_launch,
            pd_ctrl_node,
            pwm_ctrl_node,
            estimator,
            ReadyToTest(),
        ]
    )


class TestPDControlNode(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("test_pd_ctrl_node")

    def tearDown(self):
        self.node.destroy_node()

    def test_set_target_state(self):
        current_state = FreeFlyerStateStamped()

        def state_callback(msg):
            current_state.header = msg.header
            current_state.state = msg.state

        sub = self.node.create_subscription(
            FreeFlyerStateStamped,
            f"/{ROBOT_NAME}/sim/state",
            state_callback,
            10,
        )
        pub = self.node.create_publisher(
            FreeFlyerStateStamped,
            f"/{ROBOT_NAME}/ctrl/state",
            10,
        )

        try:
            # wait for nodes to start up (with 5 seconds timeout)
            end_time = time.time() + 5.0
            node_flag = False
            while time.time() < end_time and not node_flag:
                node_flag = (
                    "pd_ctrl_node" in self.node.get_node_names()
                    and "simulator_node" in self.node.get_node_names()
                )
                time.sleep(0.1)
            assert node_flag, "pd_ctrl_node or simualtor_node launch failure"

            # wait for node to initialize
            time.sleep(3.0)

            # publish target state
            target_state = FreeFlyerStateStamped()
            target_state.header.stamp = self.node.get_clock().now().to_msg()
            target_state.state.pose.x = 1.0
            target_state.state.pose.y = 1.0
            target_state.state.pose.theta = 1.0
            pub.publish(target_state)

            # wait for 15 seconds and check results
            end_time = time.time() + 15.0
            while time.time() < end_time:
                rclpy.spin_once(self.node, timeout_sec=0.1)

            # current state should be close to the target state
            self.assertAlmostEquals(current_state.state.pose.x, 1.0, delta=1e-1)
            self.assertAlmostEquals(current_state.state.pose.y, 1.0, delta=1e-1)
            self.assertAlmostEquals(current_state.state.pose.theta, 1.0, delta=1e-1)

        finally:
            self.node.destroy_subscription(sub)
            self.node.destroy_publisher(pub)
