import rclpy

import copy
import numpy as np
import typing as T

from ff_control.wrench_ctrl import WrenchController
from ff_msgs.msg import Wrench2D
from ff_msgs.msg import FreeFlyerState
from ff_msgs.msg import FreeFlyerStateStamped
from ff_params import RobotParams


class LinearController(WrenchController):
    """
    base class for any linear controller

    state definition:   [x, y, theta, vx, vy, wz]
    control definition: [fx, fy, tz]

    Note: the current implementation is not thread safe, if you want to use a
          multi-threaded exector, use the CPP version in linear_ctrl.hpp
    """

    STATE_DIM = 6
    CONTROL_DIM = 3

    def __init__(self, node_name="linear_ctrl_node"):
        super().__init__(node_name)
        self._state_sub = self.create_subscription(FreeFlyerStateStamped,
            "gt/state", self._state_callback, 10)
        self._state_ready = False
        self._state_stamped = FreeFlyerStateStamped()

    @property
    def feedback_gain_shape(self) -> T.Tuple[int, int]:
        """get shape of the feedback control gain matrix

        Returns:
            T.Tuple[int, int]: (CONTROL_DIM, STATE_DIM)
        """
        return (self.CONTROL_DIM, self.STATE_DIM)

    def get_state(self) -> T.Optional[FreeFlyerState]:
        """get the current latest state

        Returns:
            T.Optional[FreeFlyerState]: the current state, None if not available
        """
        if not self._state_ready:
            self.get_logger().error("get_state failed: state not yet ready")
            return None

        return self._state_stamped.state

    def send_control(self, state_des: T.Union[FreeFlyerState, np.ndarray], K: np.ndarray) -> None:
        """send desirable target state for linear control

        Args:
            state_des (T.Union[FreeFlyerState, np.ndarray]): desired state
            K (np.ndarray): feedback control matrix (i.e. u = Kx)
        """
        if not self._state_ready:
            self.get_logger().warn("send_control ignored, state not yet ready")
            return

        if K.shape != self.feedback_gain_shape:
            self.get_logger().error("send_control failed: incompatible gain matrix shape")
            return

        # convert desired state to vector form
        if isinstance(state_des, FreeFlyerState):
            state_des = self.state2vec(state_des)

        state_vector = self.state2vec(self.get_state())
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
        """callback invoked when the first state measurement comes in
        Sub-classes should override this function
        """
        pass

    @staticmethod
    def state2vec(state: FreeFlyerState) -> np.ndarray:
        """convert state message to state vector

        Args:
            state (FreeFlyerState): state message

        Returns:
            np.ndarray: state vector
        """
        return np.array([
            state.pose.x,
            state.pose.y,
            state.pose.theta,
            state.twist.vx,
            state.twist.vy,
            state.twist.wz,
        ])

    @staticmethod
    def vec2state(vec: np.ndarray) -> FreeFlyerState:
        """convert state vector to state message

        Args:
            vec (np.ndarray): state vector

        Returns:
            FreeFlyerState: state message
        """
        state = FreeFlyerState()
        state.pose.x = vec[0]
        state.pose.y = vec[1]
        state.pose.theta = vec[2]
        state.twist.vx = vec[3]
        state.twist.vy = vec[4]
        state.twist.wz = vec[5]

        return state

    def state_is_ready(self) -> bool:
        """check if state is ready

        Returns:
            bool: True if state is ready, False otherwise
        """
        return self._state_ready

    def _state_callback(self, msg: FreeFlyerStateStamped) -> None:
        self._state_stamped = copy.deepcopy(msg)

        if not self._state_ready:
            self._state_ready = True
            self.state_ready_callback()

