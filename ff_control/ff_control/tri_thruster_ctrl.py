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

from ff_control.ll_ctrl import LowLevelController
from ff_msgs.msg import ThrusterCommand

import numpy as np
import typing as T


class TrinaryThrusterController(LowLevelController):
    def __init__(self, node_name: str = "tri_thruster_ctrl_node") -> None:
        super().__init__(node_name)

    def set_tri_thrusters(self, tri_switches: T.Sequence[int], use_wheel: bool = False) -> None:
        """
        Convert trinary thruster commands into binary thruster commands
        """
        if use_wheel:
            self.get_logger().error("set_tri_thrusters failed: use_wheel not implemented")
            return

        if len(tri_switches) != len(ThrusterCommand().switches) / 2:
            self.get_logger().error("Incompatible thruster length sent." + str(len(tri_switches)))
            return

        switches = []
        for i in range(len(tri_switches)):
            if tri_switches[i] > 0:
                switches.extend([True, False])
            elif tri_switches[i] == 0:
                switches.extend([False, False])
            else:
                switches.extend([False, True])
        lastVal = switches.pop(-1)
        switches = [lastVal] + switches

        self.set_thrust_binary(switches)
