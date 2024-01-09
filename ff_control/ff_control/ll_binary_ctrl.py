from ff_msgs.msg import ThrusterCommand
from ff_msgs.msg import BinaryCommand
from ff_params import RobotParams

import numpy as np
import typing as T

from rclpy.node import Node


class LowLevelBinaryController(Node):
    def __init__(self, node_name: str = "ll_ctrl_node") -> None:
        super().__init__(node_name)

        # robot parameters that can be accessed by sub-classes
        self.p = RobotParams(self)

        # low level control publishers
        self._binary_thruster_pub = self.create_publisher(BinaryCommand, "ctrl/binary_command", 10)

    def set_thrust_binary(self, binary_command: T.Sequence[bool]) -> None:
        """
        Send command to set the thrusters binary output.

        :param binary_command: binary switch for each thrust
        """
        if len(binary_command) != len(BinaryCommand().binary_command):
            self.get_logger().error("Incompatible thruster length sent.")
            return
        msg = BinaryCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.binary_command = binary_command
        self._binary_thruster_pub.publish(msg)