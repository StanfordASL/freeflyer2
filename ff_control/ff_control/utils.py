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
from ff_params import RobotParams

p = RobotParams(lambda : pass)


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

def map_to_force(self, u):
    # Compute body-frame force from thrusters
    r = self.p.actuators["thrusters_lever_arm"]
    Fx = -u[0] + u[2] 
    Fy = -u[1] + u[3] 
    M = self.r * (u[0]+u[1]+u[2]+u[3])
    return Fx, Fy, M
