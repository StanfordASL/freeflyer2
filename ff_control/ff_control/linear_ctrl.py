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


import copy
import typing as T

from ff_control.wrench_ctrl import WrenchController
from ff_msgs.msg import FreeFlyerState
from ff_msgs.msg import FreeFlyerStateStamped
from ff_msgs.msg import Wrench2D
from ff_control.utils import state2vec

import numpy as np


class LinearController(WrenchController):
    """
    Base class for any linear controller.

    state definition:   [x, y, theta, vx, vy, wz]
    control definition: [fx, fy, tz]

    Note: the current implementation is not thread safe, if you want to use a
          multi-threaded exector, use the CPP version in linear_ctrl.hpp
    """

    STATE_DIM = 6
    CONTROL_DIM = 3

    def __init__(self, node_name="linear_ctrl_node"):
        super().__init__(node_name)
        self.declare_parameter("state_channel", "est/state")
        self._state_sub = self.create_subscription(
            FreeFlyerStateStamped,
            self.get_parameter("state_channel").get_parameter_value().string_value,
            self._state_callback,
            10,
        )
        self._state_ready = False
        self._state_stamped = FreeFlyerStateStamped()

    @property
    def feedback_gain_shape(self) -> T.Tuple[int, int]:
        """Get shape of the feedback control gain matrix."""
        return (self.CONTROL_DIM, self.STATE_DIM)

    def get_state(self) -> T.Optional[FreeFlyerState]:
        """Get the current latest state."""
        if not self._state_ready:
            self.get_logger().error("get_state failed: state not yet ready")
            return None

        return self._state_stamped.state

    def send_control(self, state_des: T.Union[FreeFlyerState, np.ndarray], K: np.ndarray) -> None:
        """
        Send desirable target state for linear control.

        :param state_des: desired state
        :param K: feedback control matrix (i.e. u = Kx)
        """
        if not self._state_ready:
            self.get_logger().warn("send_control ignored, state not yet ready")
            return

        if K.shape != self.feedback_gain_shape:
            self.get_logger().error("send_control failed: incompatible gain matrix shape")
            return

        # convert desired state to vector form
        if isinstance(state_des, FreeFlyerState):
            state_des = state2vec(state_des)

        state_vector = state2vec(self.get_state())
        state_delta = state_des - state_vector
        # wrap angle delta to [-pi, pi]
        state_delta[2] = (state_delta[2] + np.pi) % (2 * np.pi) - np.pi

        u = K @ state_delta

        wrench_world = Wrench2D()
        wrench_world.fx = u[0]
        wrench_world.fy = u[1]
        wrench_world.tz = u[2]
        self.set_world_wrench(wrench_world, state_vector[2])

    def state_ready_callback(self) -> None:
        """
        Get called when the first state measurement comes in.

        Sub-classes should override this function
        """
        pass

    def state_is_ready(self) -> bool:
        """
        Check if state is ready.

        :return: True if state is ready, False otherwise
        """
        return self._state_ready

    def _state_callback(self, msg: FreeFlyerStateStamped) -> None:
        self._state_stamped = copy.deepcopy(msg)

        if not self._state_ready:
            self._state_ready = True
            self.state_ready_callback()
