import rclpy
from rclpy.node import Node
from ff_msgs.msg import FreeFlyerState.msg
from ff_msgs.msg import Pose2DStamped.msg

import math
import numpy as np


# Trying to use a kalman filter to better estimate the pose
# zero acceleration model, super slow acceleration
# 

class Pose2DEstimator(Node):
    def __init__(self, node_name):
        super().__init__(node_name)

        self.min_dt = 0.05

        self.prev_state_ready = False
        self.prev_state = FreeFlyerState()

    def estimate_pose2d(self, pose_stamped):
        state = FreeFlyerState()

        state.pose = pose_stamped.pose

        if self.prev_state_ready:
            curr = pose_stamped.header.stamp
            prev = self.prev_state.header.stamp

            dt = (curr - prev).to_msg().sec + (curr - prev).to_msg().nanosec * 1e-9

            if dt < self.min_dt:
                return
            
            # Finite Difference
            vx = (pose_stamped.pose.x - self.prev_state.pose.x) / dt
            vy = (pose_stamped.pose.y - self.prev_state.pose.y) / dt

            dtheta = (pose_stamped.pose.theta - self.prev_state.pose.theta + 3 * math.pi) % (2 * math.pi) - math.pi
            wz = dtheta / dt

            alpha = self.get_parameter("lowpass_coeff").get_parameter_value().double_value
            if alpha < 0 or alpha >= 1:
                self.get_logger().error("IIR filter disabled: invalid coefficient %f", alpha)
                alpha = 0
            state.twist.vx = alpha * self.prev_state.twist.vx + (1 - alpha) * vx
            state.twist.vy = alpha * self.prev_state.twist.vy + (1 - alpha) * vy
            state.twist.wz = alpha * self.prev_state.twist.wz + (1 - alpha) * wz
        else:
            self.prev_state_ready = True

        self.prev_state.state = state
        self.prev_state.header = pose_stamped.header

        # Assuming SendStateEstimate is a function to send the state estimate
        self.send_state_estimate(state)

def main(args=None):
    rclpy.init(args=args)
    node = Pose2DEstimator("pose_2d_estimator")
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()