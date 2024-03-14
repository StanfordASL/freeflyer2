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

from ff_control.ll_ctrl import LowLevelController
from ff_msgs.msg import Wrench2D

import numpy as np


class WrenchController(LowLevelController):
    def __init__(self, node_name: str = "wrench_ctrl_node") -> None:
        super().__init__(node_name)

    def set_body_wrench(self, wrench_body: Wrench2D, use_wheel: bool = False) -> None:
        """
        Set wrench in body frame.

        :param wrench_body: wrench in body frame
        :param use_wheel: set to true to use the inertial wheel (TODO(alvin): unsupported)
        """
        if use_wheel:
            self.get_logger().error("set_wrench failed: use_wheel not implemented")
            return

        duty_cycle = np.zeros(8)
        wrench_body_clipped = self.clip_wrench(wrench_body)

        # convert force
        u_Fx = wrench_body_clipped.fx / (2 * self.p.actuators["F_max_per_thruster"])
        u_Fy = wrench_body_clipped.fy / (2 * self.p.actuators["F_max_per_thruster"])
        if u_Fx > 0:
            duty_cycle[2] = u_Fx
            duty_cycle[5] = u_Fx
        else:
            duty_cycle[1] = -u_Fx
            duty_cycle[6] = -u_Fx
        if u_Fy > 0:
            duty_cycle[4] = u_Fy
            duty_cycle[7] = u_Fy
        else:
            duty_cycle[0] = -u_Fy
            duty_cycle[3] = -u_Fy

        # convert torque
        u_M = wrench_body_clipped.tz / (
            4 * self.p.actuators["F_max_per_thruster"] * self.p.actuators["thrusters_lever_arm"]
        )
        if u_M > 0:
            duty_cycle[[1, 3, 5, 7]] += u_M
        else:
            duty_cycle[[0, 2, 4, 6]] += -u_M

        # clip to [0, 1]
        duty_cycle = np.clip(duty_cycle, 0.0, 1.0)

        self.set_thrust_duty_cycle(duty_cycle)

    def set_world_wrench(self, wrench_world: Wrench2D, theta: float) -> None:
        """
        Set wrench in world frame.

        :param wrench_world: wrench in world frame
        :param theta: rotational state of the freeflyer
        """
        cos_theta = np.cos(theta)
        sin_theta = np.sin(theta)

        wrench_body = Wrench2D()
        wrench_body.fx = cos_theta * wrench_world.fx + sin_theta * wrench_world.fy
        wrench_body.fy = -sin_theta * wrench_world.fx + cos_theta * wrench_world.fy
        wrench_body.tz = wrench_world.tz

        self.set_body_wrench(wrench_body)

    def clip_wrench(self, wrench: Wrench2D) -> Wrench2D:
        wrench_clipped = Wrench2D()
        force = np.sqrt(wrench.fx**2 + wrench.fy**2)
        force_scale = max(force / self.p.actuators["F_body_max"], 1.0)
        torque_scale = max(abs(wrench.tz) / self.p.actuators["M_body_max"], 1.0)

        wrench_clipped.fx = wrench.fx / force_scale
        wrench_clipped.fy = wrench.fy / force_scale
        wrench_clipped.tz = wrench.tz / torque_scale

        return wrench_clipped
