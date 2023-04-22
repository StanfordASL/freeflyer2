from typing import Sequence

import rclpy
from rclpy.node import Node

import numpy as np

from ff_msgs.msg import ThrusterCommand
from ff_msgs.msg import WheelVelCommand
from ff_params import RobotParams


class LowLevelController(Node):

    def __init__(self):
        super().__init__("ll_ctrl_node")
        self.p = RobotParams(self)
        self._thruster_pub = self.create_publisher(ThrusterCommand, "commands/duty_cycle", 10)
        self._wheel_pub = self.create_publisher(WheelVelCommand, "commands/velocity", 10)
        
    def set_thrust_duty_cycle(self, duty_cycle: np.ndarray) -> None:
        if len(duty_cycle) != len(ThrusterCommand().duty_cycle):
            self.get_logger().error("Incompatible thruster length sent.")
            return
        msg = ThrusterCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.duty_cycle = duty_cycle
        self._thruster_pub.publish(msg)

    def set_wheel_velocity(self, velocity: float) -> None:
        msg = WheelVelCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.velocity = velocity
        self._wheel_pub.publish(msg)
