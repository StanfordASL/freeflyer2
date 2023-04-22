from typing import Sequence

import rclpy
from rclpy.node import Node

import numpy as np

from ff_msgs.msg import Wrench2D
from ff_msgs.msg import FreeFlyerStateStamped
from ff_params import RobotParams


class LinearController(Node):

    STATE_DIM = 6
    CONTROL_DIM = 3

    def __init__(self, node_name="linear_ctrl_node"):
        super().__init__(node_name)
        self.p = RobotParams(self)
        self._wrench_body_pub = self.create_publisher(Wrench2D, "ctrl/wrench_body", 10)
        self._wrench_world_pub = self.create_publisher(Wrench2D, "ctrl/wrench_world", 10)
        self._pose_sub = self.create_subscription(FreeFlyerStateStamped, "gt/pose", self.pose_cb, 10)
        self._state_ready = False
        self._state_vector = np.zeros(self.STATE_DIM)

    def state_callback(self, msg: FreeFlyerStateStamped) -> None:
        
        pass
        
    
    def state_is_ready():
        pass

    def get_state():
        pass

    def send_control():
        pass

    def state_ready_callback():
        pass
