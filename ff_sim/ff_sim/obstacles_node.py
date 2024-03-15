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


"""
Simulates only the presence of obstacles for hardware tests.
-----------------------------------------------
"""

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray

from ff_srvs.srv import ObsInfo

class ObstaclesSimulator(Node):
    """Class to simulate only the presence of the osbtacles."""

    def __init__(self):
        super().__init__("ff_obstacles_node")
        
        # obstacles
        p_obstacles = self.declare_parameters(
            "obstacles",
            [
                ("cyl_pos_x", [1., 1.5, 2.5, 2.5]),#("cyl_pos_x", []),#
                ("cyl_pos_y", [0.7, 1.7, 0.75, 1.75]),#("cyl_pos_y", []),#
                ("cyl_rads", [0.2, 0.2, 0.2, 0.2]),#("cyl_rads", []),#
                ("cyl_heights", [0.6, 0.6, 0.6, 0.6]),#("cyl_heights", []),#
            ],
        )
        self.obstacles = {
            "cyl_pos_x": p_obstacles[0].get_parameter_value().double_array_value,
            "cyl_pos_y": p_obstacles[1].get_parameter_value().double_array_value,
            "cyl_rads": p_obstacles[2].get_parameter_value().double_array_value,
            "cyl_heights": p_obstacles[3].get_parameter_value().double_array_value,
        }
        self.obstacles_array_msg = MarkerArray()
        for n_obs in range(len(self.obstacles["cyl_rads"])):
            obs_marker = Marker()
            obs_marker.header.frame_id = 'world'
            obs_marker.header.stamp = self.get_clock().now().to_msg()
            obs_marker.ns = 'obstacle'
            obs_marker.id = n_obs
            obs_marker.type = Marker.CYLINDER
            obs_marker.action = Marker.ADD
            obs_marker.scale.x = 2*self.obstacles['cyl_rads'][n_obs]
            obs_marker.scale.y = 2*self.obstacles['cyl_rads'][n_obs]
            obs_marker.scale.z = self.obstacles['cyl_heights'][n_obs]
            obs_marker.pose.position.x = self.obstacles['cyl_pos_x'][n_obs]
            obs_marker.pose.position.y = self.obstacles['cyl_pos_y'][n_obs]
            obs_marker.pose.position.z = self.obstacles['cyl_heights'][n_obs]/2
            obs_marker.pose.orientation.x = 0.
            obs_marker.pose.orientation.y = 0.
            obs_marker.pose.orientation.z = 0.
            obs_marker.pose.orientation.w = 1.
            obs_marker.color.r = 1.
            obs_marker.color.g = 0.
            obs_marker.color.b = 0.
            obs_marker.color.a = 0.5
            self.obstacles_array_msg.markers.append(obs_marker)

        # obstacles publisher
        self.pub_obstacles = self.create_publisher(MarkerArray, f"obstacles_marker_array", 10)

        # obstale service provider
        self.obs_info_srv = self.create_service(ObsInfo, 'obstacles_info', self.obstacles_info_callback)
        # simulation params
        self.SIM_DT = 0.001
        self.sim_timer = self.create_timer(self.SIM_DT, self.sim_loop)

    def obstacles_info_callback(self, request:ObsInfo.Request, response:ObsInfo.Response) -> ObsInfo.Response:
        response.cyl_pos_x = self.obstacles["cyl_pos_x"]
        response.cyl_pos_y = self.obstacles["cyl_pos_y"]
        response.cyl_rads = self.obstacles["cyl_rads"]
        response.cyl_heights = self.obstacles["cyl_heights"]
        return response

    def sim_loop(self) -> None:
        
        # Publish
        now = self.get_clock().now().to_msg()
        for n_obs in range(len(self.obstacles['cyl_rads'])):
            self.obstacles_array_msg.markers[n_obs].header.stamp = now
        self.pub_obstacles.publish(self.obstacles_array_msg)


def main():
    rclpy.init()
    ff_obs = ObstaclesSimulator()
    rclpy.spin(ff_obs)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
