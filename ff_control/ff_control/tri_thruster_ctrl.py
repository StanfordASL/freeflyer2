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
from ff_control.utils import tri_to_bin_thrusters
from ff_msgs.msg import ThrusterCommand

import numpy as np
import typing as T


class TrinaryThrusterController(LowLevelController):
    def __init__(self, node_name: str = "tri_thruster_ctrl_node") -> None:
        super().__init__(node_name)

    def set_tri_thrusters(self, tri_switches: T.Sequence[int], use_wheel: bool = False) -> None:
        """
        Convert trinary thruster commands into binary thruster commands
        This formulation represents each thruster pair (eg thruster 1 and 2 below) as a single
        "trinary" thruster, which can either take value -1 (1 on 2 off), 0 (both off), or 1 (1 off 2 on)
        This reduces the search space for the optimization, and implicitly removes consideration of the
        undesirable case where both thrusters are on (0 net force or moment, only wasted fuel)
        tri_switches[0] = Thruster Pair [1,2]
        tri_switches[1] = Thruster Pair [3,4]
        tri_switches[2] = Thruster Pair [5,6]
        tri_switches[3] = Thruster Pair [7,0]

        
        Thrusters Configuration
             (2) e_y (1)        ___
            <--   ^   -->      /   \
           ^  |   |   |  ^     v M  )
        (3)|--o-------o--|(0)    __/
              | free- |
              | flyer |   ---> e_x
              | robot |
        (4)|--o-------o--|(7)
           v  |       |  v
            <--       -->
             (5)     (6)
        """

        if len(tri_switches) != len(ThrusterCommand().switches) / 2:
            self.get_logger().error("Incompatible thruster length sent." + str(len(tri_switches)))
            return

        switches = tri_to_bin_thrusters(tri_switches)

        self.set_thrust_binary(switches)
