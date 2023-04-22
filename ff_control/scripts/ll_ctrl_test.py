#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np

from ff_control.ll_ctrl import LowLevelController


class TestLowLevelController(LowLevelController):

    def __init__(self):
        super().__init__()
        self.timer = self.create_timer(1.0, self.set_thrust_command)
    
    def set_thrust_command(self) -> None:
        self.set_thrust_duty_cycle(np.ones(8) * 0.5)


def main(args=None):
    rclpy.init(args=args)
    debug_ctrl = TestLowLevelController()
    rclpy.spin(debug_ctrl)
    rclpy.shutdown()


if __name__ == "__main__":
    main()