#!/usr/bin/env python
# MIT License
#
# Copyright (c) 2024 Stanford Autonomous Systems Lab
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


"""
Injects goal position into ROS stack for testing
-----------------------------------------------
"""
from ff_msgs.msg import FreeFlyerStateStamped
from rclpy.node import Node
import rclpy
import math
import sys

goal_positions = [[1.0,1.0,math.pi], [0.5,0.5,0.0]]
goal_velos = [[0.0,0.0,0.0], [0.0,0.0,0.0]]

class GoalPublisherNode(Node):
    def __init__(self):
        super().__init__("goal_publisher")
        self.pub_goal = self.create_publisher(FreeFlyerStateStamped, f"robot/ctrl/state", 10)

    def publish_goal(self, pos, velo):
        goal_pose = FreeFlyerStateStamped()
        goal_pose.state.pose.x = pos[0]
        goal_pose.state.pose.y = pos[1]
        goal_pose.state.pose.theta = pos[2]
        goal_pose.state.twist.vx = velo[0]
        goal_pose.state.twist.vy = velo[1]
        goal_pose.state.twist.wz = velo[2]           

        self.pub_goal.publish(goal_pose)

def main():
    index = int(sys.argv[1])
    rclpy.init()
    publisher = GoalPublisherNode()
    publisher.publish_goal(goal_positions[index], goal_velos[index])
    rclpy.spin(publisher)
    rclpy.shutdown()

if __name__ == "__main__":
    main()