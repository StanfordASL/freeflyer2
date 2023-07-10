#!/usr/bin/env python3

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
