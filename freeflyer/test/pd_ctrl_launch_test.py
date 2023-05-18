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
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_testing.actions import ReadyToTest

import rclpy


ROBOT_NAME = 'freeflyer'


def generate_test_description():
    sim_launch = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('ff_sim'),
            'launch',
            'single.launch.py',
        ]),
        launch_arguments={'robot_name': ROBOT_NAME}.items(),
    )
    pd_ctrl_node = Node(
        package='ff_control',
        executable='pd_ctrl_node',
        name='pd_ctrl_node',
        namespace=ROBOT_NAME,
    )

    return LaunchDescription([
        sim_launch,
        pd_ctrl_node,
        ReadyToTest(),
    ]), {'sim_launch': sim_launch, 'pd_ctrl_node': pd_ctrl_node}


class TestPDControlNode(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_pd_ctrl_node')

    def tearDown(self):
        self.node.destroy_node()

    def test_set_target_state(self, launch_service):
        current_state = FreeFlyerStateStamped()

        def state_callback(msg):
            current_state.header = msg.header
            current_state.state = msg.state

        sub = self.node.create_subscription(
            FreeFlyerStateStamped,
            f'/{ROBOT_NAME}/gt/state',
            state_callback,
            10,
        )
        pub = self.node.create_publisher(
            FreeFlyerStateStamped,
            f'/{ROBOT_NAME}/ctrl/state',
            10,
        )

        try:
            # wait for nodes to start up (with 5 seconds timeout)
            end_time = time.time() + 5.
            node_flag = False
            while time.time() < end_time and not node_flag:
                node_flag = 'pd_ctrl_node' in self.node.get_node_names() and \
                            'simulator_node' in self.node.get_node_names()
                time.sleep(0.1)
            assert node_flag, 'pd_ctrl_node or simualtor_node launch failure'

            # wait for node to initialize
            time.sleep(3.)

            # publish target state
            target_state = FreeFlyerStateStamped()
            target_state.header.stamp = self.node.get_clock().now().to_msg()
            target_state.state.pose.x = 1.
            target_state.state.pose.y = 1.
            target_state.state.pose.theta = 1.
            pub.publish(target_state)

            # wait for 15 seconds and check results
            end_time = time.time() + 15.
            while time.time() < end_time:
                rclpy.spin_once(self.node, timeout_sec=0.1)

            # current state should be close to the target state
            self.assertAlmostEquals(current_state.state.pose.x, 1., delta=1e-1)
            self.assertAlmostEquals(current_state.state.pose.y, 1., delta=1e-1)
            self.assertAlmostEquals(current_state.state.pose.theta, 1., delta=1e-1)

        finally:
            self.node.destroy_subscription(sub)
            self.node.destroy_publisher(pub)
