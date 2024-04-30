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


"""
Simulates the freeflyer.

Maps thrusters + wheel control input
to a state evolution over time
-----------------------------------------------
"""

import math
import sys
import numpy as np
import typing as T

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

from ff_msgs.msg import (
    FreeFlyerStateStamped,
    Wrench2DStamped,
    WheelVelCommand,
    ThrusterCommand,
    ControllerMetrics,
)

from ff_params import RobotParams


class ControllerMetricsPublisher(Node):
    """
    Class to listen to free flyer commands and calculate metrics
    Calculates two key metrics:
    1. total_gas_time: Measures total time that thrusters are on (summed over each thruster). Provides
        a proxy for total gas expenditure over time
    2. running_duty_cycles: Measures average duty cycle for each thruster over a time window specified by
        self.duty_cycle_window.

    """

    def __init__(self):
        super().__init__("ff_ctrl_metrics")
        self.curr_time = self.get_clock().now()
        self.steps = 0
        self.running_total_gas = 0
        self.prev_thruster_sum = 0
        self.thrust_hist = [[] for i in range(8)]
        self.time_hist = []
        self.thrust_duty_cycles = [0] * 8
        self.duty_cycle_window = 6
        self.rolled_up = False

        self.sub_wheel_cmd_vel = self.create_subscription(
            WheelVelCommand, "commands/velocity", self.process_new_wheel_cmd, 10
        )
        self.sub_thrusters_cmd_binary = self.create_subscription(
            ThrusterCommand, "commands/binary_thrust", self.process_new_binary_thrust_cmd, 10
        )
        self.pub_controller_metrics = self.create_publisher(
            ControllerMetrics, "metrics/controller", 10
        )

    def process_new_wheel_cmd(self, msg: WheelVelCommand) -> None:
        """Placeholder for now"""
        pass

    def process_new_binary_thrust_cmd(self, msg: ThrusterCommand) -> None:
        """Process binary thrusters"""
        now = self.get_clock().now()
        dt = (now - self.curr_time).nanoseconds / 1e9

        # Perform Euler integration for how long each thruster was on
        thrusters = np.array(msg.switches, dtype=float)
        self.running_total_gas += self.prev_thruster_sum * dt
        self.prev_thruster_sum = np.sum(thrusters)

        # Calculate rolling average of duty cycle for each thruster
        self.steps += 1
        if not self.rolled_up:  # Ensure valid duty cycles at the beginning
            self.time_hist.append(dt)
            for i in range(8):
                self.thrust_hist[i].append(thrusters[i])
                self.thrust_duty_cycles[i] = np.dot(self.thrust_hist[i], self.time_hist) / np.sum(
                    self.time_hist
                )
            if self.steps >= self.duty_cycle_window:
                self.rolled_up = True
        else:  # Once queue is filled up, can just treat this as a list that's constantly being updawted
            self.time_hist.pop(0)
            self.time_hist.append(dt)
            for i in range(8):
                self.thrust_hist[i].pop(0)
                self.thrust_hist[i].append(thrusters[i])
                self.thrust_duty_cycles[i] = np.dot(self.thrust_hist[i], self.time_hist) / np.sum(
                    self.time_hist
                )

        metrics = ControllerMetrics()
        metrics.header.stamp = now
        metrics.total_gas_time = self.running_total_gas
        metrics.running_duty_cycles = self.thrust_duty_cycles
        self.pub_controller_metrics.publish(metrics)

        self.curr_time = now


def main():
    rclpy.init()
    ff_metrics = ControllerMetricsPublisher()
    rclpy.spin(ff_metrics)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
