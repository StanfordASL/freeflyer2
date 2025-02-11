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


import typing as T

import numpy as np
from rcl_interfaces.srv import GetParameters
import rclpy
from rclpy.node import Node


class RobotParams:
    def __init__(self, node: Node, param_ready_cb: T.Optional[T.Callable[[], None]] = None):
        robot_name = node.get_namespace()
        cli = node.create_client(GetParameters, f"{robot_name}/robot_params_node/get_parameters")
        self.node = node
        self.param_ready_cb = param_ready_cb

        while not cli.service_is_ready():
            cli.wait_for_service(5.0)
            node.get_logger().info("parameter service not ready, retrying...")

        self.request = GetParameters.Request()
        self.request.names = [
            "dynamics.mass",
            "dynamics.inertia",
            "dynamics.radius",
            "dynamics.CoM_offset",
            "dynamics.force_const",
            "actuators.F_max_per_thruster",
            "actuators.thrusters_lever_arm",
            "actuators.F_body_max",
            "actuators.M_body_max",
            "actuators.min_inp_percent",
            "actuators.max_inp_percent",
            "actuators.gamma_min",
            "actuators.gamma_max",
        ]

        future = cli.call_async(self.request)
        future.add_done_callback(self._param_ready_cb)

        self.loaded = False

    def _param_ready_cb(self, future: rclpy.task.Future):
        try:
            res = future.result()
        except Exception as e:
            self.node.get_logger().error("get robot params failed: %r" % (e,))
        else:
            self.dynamics = {
                "mass": res.values[0].double_value,
                "inertia": res.values[1].double_value,
                "radius": res.values[2].double_value,
                "CoM_offset": np.array(res.values[3].double_array_value),
                "force_const": np.array(res.values[4].double_array_value),
            }

            self.actuators = {
                "F_max_per_thruster": res.values[5].double_value,
                "thrusters_lever_arm": res.values[6].double_value,
                "F_body_max": res.values[7].double_value,
                "M_body_max": res.values[8].double_value,
                "min_inp_percent": res.values[9].double_value,
                "max_inp_percent": res.values[10].double_value,
                "gamma_min": res.values[11].double_value,
                "gamma_max": res.values[12].double_value,
            }

            self.loaded = True

            if self.param_ready_cb is not None:
                self.param_ready_cb()
