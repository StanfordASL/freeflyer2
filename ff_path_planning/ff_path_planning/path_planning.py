#!/usr/bin/env python3

import time
import traceback
import json
import math

import rclpy
from rclpy.node import Node
from ff_srvs.srv import PathPlan
from multiprocessing import Process, set_start_method

import numpy as np

from pmpc import solve, Problem

from .import_costs_and_dynamics import DYNAMICS_MODULES, COSTS_MODULES

####################################################################################################


def to_ros_array(x):
    return x.reshape(-1).tolist()


def example_request_for_jit():
    """Send an example request to the path planning service to trigger JIT compilation."""
    rclpy.init()
    node = Node("path_planning_client")
    node.get_logger().info("Starting an example request node")
    cli = node.create_client(PathPlan, "path_planning")
    while not cli.wait_for_service(timeout_sec=0.1):
        pass
    node.get_logger().info("Path Planning Service is up")
    request = PathPlan.Request()
    request.dynamics = "single_integrator"
    secs, nsecs = rclpy.clock.Clock().now().seconds_nanoseconds()
    request.t0 = secs + nsecs / 1e9
    request.x0 = to_ros_array(np.array([5.0]))

    t = time.time()
    node.get_logger().info("Sending compilation request")
    future = cli.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    result = future.result()
    node.get_logger().info(f"Precompilation took {time.time() - t:.4} seconds")

    N = request.horizon
    ts = np.array(result.times)
    X = np.array(result.states).reshape((N + 1, -1))
    U = np.array(result.controls).reshape((N, -1))
    L = np.array(result.feedback).reshape((N, X.shape[-1], U.shape[-1]))
    node.destroy_node()
    rclpy.shutdown()


####################################################################################################


class PathPlanningService(Node):
    def __init__(self):
        """A path planning service node based on the `pmpc` library."""
        super().__init__("path_planning")
        print("Starting the path planning service")
        self.srv = self.create_service(PathPlan, "path_planning", self.compute_plan)
        self.get_logger().info("Available dynamics: [" + ", ".join(DYNAMICS_MODULES.keys()) + "]")
        self.get_logger().info("Available costs:    [" + ", ".join(COSTS_MODULES.keys()) + "]")

    def _empty_plan(self, request, response):
        """Construct and empty plan, filled with NaNs to indicate failure."""
        xdim, udim, N = request.xdim, request.udim, request.horizon
        xdim = xdim if xdim > 0 else 1
        udim = udim if udim > 0 else 1
        response.times = to_ros_array(math.nan * np.ones(N + 1))
        response.states = to_ros_array(math.nan * np.ones((N + 1, xdim)))
        response.controls = to_ros_array(math.nan * np.ones((N, udim)))
        response.feedback = to_ros_array(math.nan * np.ones((N, xdim, udim)))
        return response

    def _resolve_dimensions(self, request):
        """Resolve the dimensions, [xdim, udim] based on provided values or the dynamics module."""
        dyn_mod = DYNAMICS_MODULES[request.dynamics]
        xdim, udim = request.xdim, request.udim
        if hasattr(dyn_mod, "XDIM") or hasattr(dyn_mod, "xdim"):
            mod_xdim = getattr(dyn_mod, "XDIM", getattr(dyn_mod, "xdim", None))
            if request.xdim > 0:
                assert request.xdim == mod_xdim
            xdim = mod_xdim
        if hasattr(dyn_mod, "UDIM") or hasattr(dyn_mod, "udim"):
            mod_udim = getattr(dyn_mod, "UDIM", getattr(dyn_mod, "udim", None))
            if request.udim > 0:
                assert request.udim == mod_udim
            udim = mod_udim
        assert xdim > 0 and udim > 0
        return dict(xdim=xdim, udim=udim, N=request.horizon)

    def compute_plan(self, request, response):
        """Main service callback for computing the optimal plan."""
        # validate the request ###################
        if request.dynamics not in DYNAMICS_MODULES or request.cost not in COSTS_MODULES:
            msg = f"Dynamics `{request.dynamics}` or cost `{request.cost}` not found"
            msg += "\nDynamics available: " + ", ".join(DYNAMICS_MODULES.keys())
            msg += "\nCosts available: " + ", ".join(COSTS_MODULES.keys())
            self.get_logger().error(msg)
            return self._empty_plan(request, response)
        try:
            dims = self._resolve_dimensions(request)
        except AssertionError:
            self.get_logger().error("Invalid dimensions: xdim, udim")
            return self._empty_plan(request, response)
        # validate the request ###################

        # parse extra problem parameters #########
        params = None
        try:
            if len(request.params_json) > 0:
                params = json.loads(request.params_json)
        except ValueError:
            self.get_logger().error("Error deserializing params_json")
            self.get_logger().error(traceback.format_exc())
            return self._empty_plan(request, response)
        # parse extra problem parameters #########

        # construct the problem ##################
        try:
            if params is None:
                cost = COSTS_MODULES[request.cost].cost(**dims)
            else:
                cost = COSTS_MODULES[request.cost].cost(**dims, params=params)
            f_fx_fu_fn = DYNAMICS_MODULES[request.dynamics].f_fx_fu_fn
            x0 = np.array(request.x0)
            p = Problem(x0=x0, f_fx_fu_fn=f_fx_fu_fn, **dims, **cost)
            p.max_it = request.max_it
            p.verbose = True
        except:
            self.get_logger().error(traceback.format_exc())
            return self._empty_plan(request, response)
        # construct the problem ##################

        # solve ##################################
        try:
            X, U, _ = solve(**p)
        except:
            self.get_logger().error(traceback.format_exc())
            return self._empty_plan(request, response)
        # solve ##################################

        # fill the response ######################
        response.times = to_ros_array(request.t0 + np.arange(request.horizon + 1) * request.dt)
        response.states = to_ros_array(X)
        response.controls = to_ros_array(U)
        response.feedback = to_ros_array(
            math.nan * np.ones((dims["N"], dims["xdim"], dims["udim"]))
        )
        # fill the response ######################
        self.get_logger().info("Path planning service finished")
        return response


####################################################################################################


def main():
    # send an example request to trigger the JIT compilation immediately
    set_start_method("spawn")
    p = Process(target=example_request_for_jit)
    p.start()

    # spin the node up
    rclpy.init()
    path_planning_service = PathPlanningService()
    rclpy.spin(path_planning_service)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
