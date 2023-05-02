#!/usr/bin/env python3

import sys
import os
from pathlib import Path
from importlib import import_module

print(sys.executable)

from ff_srvs.srv import PathPlan

import rclpy
from rclpy.node import Node

from pmpc import solve

all_dynamics = [
    x
    for x in os.listdir(Path(__file__).absolute().parent / "dynamics")
    if Path(x).suffix == ".py" and x != "__init__.py"
]
all_costs = [
    x
    for x in os.listdir(Path(__file__).absolute().parent / "costs")
    if Path(x) == ".py" and x != "__init__.py"
]

class MinimalService(Node):
    def __init__(self):
        super().__init__("path_planning")
        self.srv = self.create_service(PathPlan, "path_planning", self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info("Incoming request\na: %d b: %d" % (request.a, request.b))

        return response


def main():
    rclpy.init()
    minimal_service = MinimalService()
    rclpy.spin(minimal_service)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
