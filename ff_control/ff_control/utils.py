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

import numpy as np
from ff_msgs.msg import FreeFlyerState


##################### Helper Functions to unpack FreeFlyerState #####################
def state2vec(state: FreeFlyerState) -> np.ndarray:
    """
    Convert state message to state vector.

    :param state: state message
    :return: state vector
    """
    return np.array(
        [
            state.pose.x,
            state.pose.y,
            state.pose.theta,
            state.twist.vx,
            state.twist.vy,
            state.twist.wz,
        ]
    )


def vec2state(vec: np.ndarray) -> FreeFlyerState:
    """
    Convert state vector to state message.

    :param vec: state vector
    :return: state message
    """
    state = FreeFlyerState()
    state.pose.x = vec[0]
    state.pose.y = vec[1]
    state.pose.theta = vec[2]
    state.twist.vx = vec[3]
    state.twist.vy = vec[4]
    state.twist.wz = vec[5]

    return state


def map_to_force(u, r):
    # Compute body-frame force from trinary thrusters
    Fx = -u[0] + u[2]
    Fy = -u[1] + u[3]
    M = r * (u[0] + u[1] + u[2] + u[3])
    return Fx, Fy, M


def tri_to_bin_thrusters(tri_switches):
    """
    In: 4 Trinary thrusters (-1, 0, 1)
    Out: 8 Binary thrusters (0, 1) mapped to hardware
    Convert trinary thruster commands into binary thruster commands
    This formulation represents each thruster pair (eg thruster 1 and 2 below) as a single
    "trinary" thruster, which can either take value 1 (1 on 2 off), 0 (both off), or -1 (1 off 2 on)
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
    return switches


def bin_to_tri_thrusters(bin_switches):
    """
    In: 4 Trinary thrusters (-1, 0, 1)
    Out: 8 Binary thrusters (0, 1) mapped to hardware
    Convert trinary thruster commands into binary thruster commands
    This formulation represents each thruster pair (eg thruster 1 and 2 below) as a single
    "trinary" thruster, which can either take value 1 (1 on 2 off), 0 (both off), or -1 (1 off 2 on)
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
    if len(bin_switches) % 2 != 0:
        raise ValueError("Invalid size of binary switches")

    tri_switches = []
    for i in range(len(bin_switches) // 2):
        ind1 = 2 * i + 1
        ind2 = (2 * i + 2) % len(bin_switches)
        if bin_switches[ind1] and not bin_switches[ind2]:
            tri_switches.append(1)
        elif not bin_switches[ind1] and bin_switches[ind2]:
            tri_switches.append(-1)
        elif not bin_switches[ind1] and not bin_switches[ind2]:
            tri_switches.append(0)
        else:
            raise ValueError("Invalid set of binary switches! Not complementary")
    return tri_switches
