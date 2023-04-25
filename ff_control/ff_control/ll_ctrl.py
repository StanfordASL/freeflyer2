import rclpy
from rclpy.node import Node

import numpy as np

from ff_msgs.msg import ThrusterCommand
from ff_msgs.msg import WheelVelCommand
from ff_params import RobotParams


class LowLevelController(Node):

    def __init__(self, node_name: str = "ll_ctrl_node") -> None:
        super().__init__(node_name)

        # robot parameters that can be accessed by sub-classes
        self.p = RobotParams(self)

        # low level control publishers
        self._thruster_pub = self.create_publisher(ThrusterCommand, "ctrl/duty_cycle", 10)
        self._wheel_pub = self.create_publisher(WheelVelCommand, "ctrl/velocity", 10)

    def set_thrust_duty_cycle(self, duty_cycle: np.ndarray) -> None:
        """send command to set the thrusters duty cycles

        Args:
            duty_cycle (np.ndarray): duty cycle for each thrust (in [0, 1])
        """
        if len(duty_cycle) != len(ThrusterCommand().duty_cycle):
            self.get_logger().error("Incompatible thruster length sent.")
            return
        msg = ThrusterCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.duty_cycle = duty_cycle
        self._thruster_pub.publish(msg)

    def set_wheel_velocity(self, velocity: float) -> None:
        """send command to set the inertial wheel velocity

        TODO(alvin): suppor this or remove?

        Args:
            velocity (float): angular velocity in [rad/s]
        """
        msg = WheelVelCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.velocity = velocity
        self._wheel_pub.publish(msg)

