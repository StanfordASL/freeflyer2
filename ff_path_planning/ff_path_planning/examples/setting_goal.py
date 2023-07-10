#!/usr/bin/env python3

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

"""Broadcast a simple static goal to the controller."""

import rclpy
from rclpy.node import Node
from ff_msgs.msg import FreeFlyerStateStamped, FreeFlyerState, Twist2D, Pose2D as FF_Pose2D

####################################################################################################


class SimpleGoalNode(Node):
    def __init__(self):
        """A path planning service node based on the `pmpc` library."""
        super().__init__("simple_goal")
        self.ctrl_state_pub = self.create_publisher(FreeFlyerStateStamped, "ctrl/state", 10)
        self.state_sub = self.create_subscription(
            FreeFlyerStateStamped, "gt/state", self.state_cb, 10
        )
        self.state, self.goal_pose = None, None

        self.already_published = False
        self.create_timer(1.0, self.publish_goal)

        pub = self.create_publisher(FreeFlyerStateStamped, "state_init", 10)
        msg = FreeFlyerStateStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.state = FreeFlyerState(twist=Twist2D(), pose=FF_Pose2D())
        pub.publish(msg)

    def state_cb(self, state):
        if self.goal_pose is None:
            x, y, th = state.state.pose.x, state.state.pose.y, state.state.pose.theta
            self.goal_pose = FreeFlyerStateStamped()
            self.goal_pose.header.stamp = self.get_clock().now().to_msg()
            self.goal_pose.state = FreeFlyerState(
                pose=FF_Pose2D(x=x + 1, y=y + 2, theta=th), twist=Twist2D()
            )
        self.state = state

    def publish_goal(self):
        # if self.already_published:
        #    return
        if self.goal_pose is None:
            return
        self.goal_pose.header.stamp = self.get_clock().now().to_msg()
        self.ctrl_state_pub.publish(self.goal_pose)
        self.get_logger().info("Published goal pose")
        print(f"Published goal pose: {self.goal_pose.state.pose.x}, {self.goal_pose.state.pose.y}")
        print(f"Actual pose {self.state.state.pose.x}, {self.state.state.pose.y}")
        self.already_published = True


####################################################################################################


def main():
    # spin the node up
    rclpy.init()
    path_planning_service = SimpleGoalNode()
    rclpy.spin(path_planning_service)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
