#!/usr/bin/env python3

from copy import copy
import json
import math

import rclpy
from rclpy.node import Node
from ff_srvs.srv import PathPlan
from ff_msgs.msg import FreeFlyerStateStamped, FreeFlyerState, Twist2D, Pose2D as FF_Pose2D
from geometry_msgs.msg import PoseStamped, Pose, Pose2D, Point, Quaternion
from nav_msgs.msg import Path

import numpy as np
from scipy.interpolate import interp1d

####################################################################################################


def pose2D_to_pose(pose2D):
    pose = Pose(position=Point(x=pose2D.x, y=pose2D.y, z=0.0))
    # Convert the theta value (2D orientation) to a quaternion (3D orientation)
    half_theta = pose2D.theta / 2
    pose.orientation = Quaternion(x=0.0, y=0.0, z=math.sin(half_theta), w=math.cos(half_theta))
    return pose


def to_ros_array(x):
    return x.reshape(-1).tolist()


def add_time(sec: int, nsec: int, duration: float):
    """Add a duration to a time."""
    sec += int(duration)
    nsec += int((duration - int(duration)) * 1e9)
    if nsec >= 1e9:
        sec, nsec = sec + 1, int(nsec - 1e9)
    return sec, nsec


####################################################################################################


class NavigationNode(Node):
    def __init__(self):
        """A path planning service node based on the `pmpc` library."""
        super().__init__("simple_goal")
        self.ctrl_state_pub = self.create_publisher(FreeFlyerStateStamped, "ctrl/state", 10)
        self.state_sub = self.create_subscription(
            FreeFlyerStateStamped, "gt/state", self.state_cb, 10
        )
        self.state, self.goal_pose = None, None
        self.path_plan_client = self.create_client(PathPlan, "/path_planning")
        while not self.path_plan_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")

        self.marker_pub = self.create_publisher(Path, "path_marker", 10)

        self.computing_plan, self.plan = False, None
        self.plan_futures = []
        self.create_timer(1.0, self.recompute_plan_cb)
        self.create_timer(1.0, self.publish_goal_cb)

        self.get_logger().info("Navigation node is up")

        pub = self.create_publisher(FreeFlyerStateStamped, "state_init", 10)
        msg = FreeFlyerStateStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.state = FreeFlyerState(twist=Twist2D(), pose=FF_Pose2D())
        self.get_logger().info("Publishing initial state")
        pub.publish(msg)

    def show_path(self) -> None:
        """Publish the path as a path marker."""
        if self.plan is None:
            return
        X = self.plan["X"]
        marker = Path()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.poses = []
        t0 = self.plan["t"][0]
        for i in range(X.shape[0]):
            pose = PoseStamped()
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = X[i, 0], X[i, 1], 0.5
            pose.header.frame_id = marker.header.frame_id

            time_stamp = copy(marker.header.stamp)
            sec, nsec = add_time(time_stamp.sec, time_stamp.nanosec, self.plan["t"][i] - t0)
            time_stamp.sec, time_stamp.nanosec = sec, nsec
            pose.header.stamp = time_stamp

            marker.poses.append(pose)
        self.marker_pub.publish(marker)

    def loop(self) -> None:
        """Loop until shutdown, pulling plans if they are available."""
        while rclpy.ok():
            rclpy.spin_once(self)

            # attempt to parse returned planning service calls
            last_future, incomplete_futures = None, []
            for i in range(len(self.plan_futures)):
                last_future = self.plan_futures[i]
                if not last_future.done():
                    incomplete_futures.append(last_future)
            self.plan_futures = incomplete_futures
            if last_future is None:
                continue

            # check if the service call is not None
            result = last_future.result()
            if result is None:
                self.get_logger().info("No plan found")
                continue

            # parse the plan from the service call
            ts = np.array(result.times)
            N = len(ts) - 1
            X = np.array(result.states).reshape((N + 1, -1))
            U = np.array(result.controls).reshape((N, -1))
            L = np.array(result.feedback).reshape((N, X.shape[-1], U.shape[-1]))
            self.plan = dict(t=ts, X=X, U=U, L=L)
            self.show_path()

    def make_goal_pose(self, x: float, y: float) -> FreeFlyerStateStamped:
        """Create a goal pose from the current state, with the given x and y coordinates."""
        th = self.state.state.pose.theta
        goal_pose = FreeFlyerStateStamped()
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.state = FreeFlyerState(pose=FF_Pose2D(x=x, y=y, theta=th), twist=Twist2D())
        return goal_pose

    def state_cb(self, state) -> None:
        """Read the state as it becomes available."""
        self.state = state

    def get_current_time(self) -> float:
        secs, nsecs = rclpy.clock.Clock().now().seconds_nanoseconds()
        return secs + nsecs / 1e9

    def publish_goal_cb(self) -> None:
        """Publish the goal pose based on the interpolation of the plan."""
        if self.plan is None:
            self.get_logger().info("No plan available yet.")
            return
        current_time = self.get_current_time()
        if current_time < self.plan["t"][0] or current_time > self.plan["t"][-1]:
            self.get_logger().info("Current time outside of the plan time horizon.")
        x_goal = interp1d(self.plan["t"], self.plan["X"], axis=0)(current_time)
        goal_pose = self.make_goal_pose(x_goal[0], x_goal[1])
        self.ctrl_state_pub.publish(goal_pose)
        self.get_logger().info("Published goal pose")

    def recompute_plan_cb(self) -> None:
        request = PathPlan.Request(horizon=20)

        request.dynamics = "single_integrator_2D"
        request.t0 = self.get_current_time()
        x, y = self.state.state.pose.x, self.state.state.pose.y
        request.x0 = to_ros_array(np.array([x, y]))

        # we'll provide extra arguments to the cost function, which accepts params encoded as a JSON
        request.cost = "my_cost"
        X_ref = np.tile(np.array([5, 5]), (request.horizon, 1)).tolist()
        R = np.tile(np.eye(2) * 1e2, (request.horizon, 1, 1)).tolist()
        request.params_json = json.dumps(dict(X_ref=X_ref, R=R))

        # send a request to the path planner
        self.plan_futures.append(self.path_plan_client.call_async(request))


####################################################################################################


def main():
    # spin the node up
    rclpy.init()
    navigation_node = NavigationNode()
    navigation_node.loop()
    rclpy.shutdown()


if __name__ == "__main__":
    main()