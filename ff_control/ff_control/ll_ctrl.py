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

import numpy as np
import typing as T

from ff_msgs.msg import ThrusterPWMCommand, ThrusterBinaryCommand
from ff_msgs.msg import WheelVelCommand
from ff_params import RobotParams
from rclpy.node import Node


class LowLevelController(Node):
    def __init__(self, node_name: str = "ll_ctrl_node") -> None:
        super().__init__(node_name)

        # robot parameters that can be accessed by sub-classes
        self.p = RobotParams(self)

        # low level thruster control publishers
        self._thruster_pwm_pub = self.create_publisher(ThrusterPWMCommand, "ctrl/duty_cycle", 10)
        self._thruster_binary_pub = self.create_publisher(
            ThrusterBinaryCommand, "ctrl/binary_thrust", 10
        )

        self._wheel_pub = self.create_publisher(WheelVelCommand, "ctrl/velocity", 10)

    def set_thrust_duty_cycle(self, duty_cycle: np.ndarray) -> None:
        """
        Send command to set the thrusters duty cycles.

        :param duty_cycle: duty cycle for each thrust (in [0, 1])
        """
        if len(duty_cycle) != len(ThrusterPWMCommand().duty_cycles):
            self.get_logger().error("Incompatible thruster length sent.")
            return
        msg = ThrusterPWMCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.duty_cycles = duty_cycle
        self._thruster_pwm_pub.publish(msg)

    def set_thrust_binary(self, switches: T.Sequence[bool]) -> None:
        """
        Send command to set the thrusters binary output.

        :param switches: binary switch for each thrust
        """
        if len(switches) != len(ThrusterBinaryCommand().switches):
            self.get_logger().error("Incompatible thruster length sent.")
            return
        msg = ThrusterBinaryCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.switches = switches
        self._thruster_binary_pub.publish(msg)

    def set_wheel_velocity(self, velocity: float) -> None:
        """
        Send command to set the inertial wheel velocity.

        TODO(alvin): suppor this or remove?

        :param velocity: angular velocity in [rad/s]
        """
        msg = WheelVelCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.velocity = velocity
        self._wheel_pub.publish(msg)
