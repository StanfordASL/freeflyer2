#!/usr/bin/env python3

import time
import json

import numpy as np

from ff_srvs.srv import PathPlan
import rclpy
from rclpy.node import Node

####################################################################################################


def to_ros_array(x):
    return x.reshape(-1).tolist()


def _send_request(node, cli):
    request = PathPlan.Request()
    request.dynamics = "single_integrator"
    secs, nsecs = rclpy.clock.Clock().now().seconds_nanoseconds()
    request.t0 = secs + nsecs / 1e9
    request.x0 = to_ros_array(np.array([5.0]))
    # example json params, numpy arrays can be sent as (nested list)
    # use: `arr.tolist()``
    request.params_json = json.dumps(
        dict(dt=0.1, data=(1, dict(a=1, b=2, x=np.array([1.0]).tolist())))
    )
    future = cli.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    result = future.result()

    N = request.horizon
    ts = np.array(result.times)
    X = np.array(result.states).reshape((N + 1, -1))
    U = np.array(result.controls).reshape((N, -1))
    L = np.array(result.feedback).reshape((N, X.shape[-1], U.shape[-1]))
    return ts, X, U, L


class ExamplePathPlanningClient(Node):
    def __init__(self):
        super().__init__("path_planning_client")
        self.cli = self.create_client(PathPlan, "path_planning")
        while not self.cli.wait_for_service(timeout_sec=0.1):
            self.get_logger().info("service not available, waiting again...")

    def send_request(self):
        print("Sending a request to the path planning service")
        t = time.time()
        ret = _send_request(self, self.cli)
        self.get_logger().info(f"Calling the path planning took {time.time() - t:.4e} seconds")
        return ret


####################################################################################################


def main():
    rclpy.init()

    example_client = ExamplePathPlanningClient()
    ts, X, U, L = example_client.send_request()
    example_client.get_logger().info(f"ts = {ts}")
    example_client.get_logger().info(f"X = {X}")
    example_client.get_logger().info(f"U = {U}")
    example_client.get_logger().info(f"L = {L}")

    example_client.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
