import rclpy
from rclpy.node import Node
import numpy as np
import time

from ff_msgs.msg import ThrusterCommand
from ff_msgs.msg import WheelVelCommand
from std_msgs.msg import Bool

class SafetyFilter(Node):

    def __init__(self, check_period=5):
        super().__init__('safety_filter')
        self.get_logger().info("Safety Filter Node Started")
        self.last_thrust_time = None
        self.last_wheel_time = None
        self.wait_period = check_period # seconds to wait w/ receiving anything before publishing a zero message
        self.kill_state = False

        # Create publishers for thruster and wheel commands
        self.thrust_pub = self.create_publisher(ThrusterCommand, 'commands/duty_cycle', 10)
        self.wheel_pub = self.create_publisher(WheelVelCommand, 'commands/velocity', 10)

        # Create subscribers for thruster and wheel commands
        self.thrust_sub = self.create_subscription(
            ThrusterCommand,
            'ctrl/duty_cycle',
            self.thrust_callback,
            10)
        self.wheel_sub = self.create_subscription(
            WheelVelCommand,
            'ctrl/velocity',
            self.wheel_callback,
            10)
        
        # Create timer to check if we've received any messages in the last 'wait_period' seconds
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.check_timer_callback)

        # Create a subscriber to listen for a kill message
        self.kill_sub = self.create_subscription(
            Bool,
            'kill',
            self.kill_callback,
            10)

    def kill_callback(self, msg):
        if msg.data:
            self.get_logger().info("Kill message received. Publishing zero thrust and wheel commands.")
            self.send_zero_thrust()
            self.send_zero_wheel()
            self.kill_state = True
    
    def check_timer_callback(self):
        # If no message was received in the last 'wait_period' seconds, publish a zero message
        if self.last_thrust_time is not None and self.get_clock().now() - self.last_thrust_time > self.wait_period:
            self.send_zero_thrust()
            self.last_thrust_time = self.get_clock().now()

        if self.last_wheel_time is not None and self.get_clock().now() - self.last_wheel_time > self.wait_period:
            self.send_zero_wheel()
            self.last_wheel_time = self.get_clock().now()

    def thrust_callback(self, msg):
        if self.kill_state:
            self.get_logger().info("Kill state is active. Ignoring thrust command.")
            return
        # Publish the message to thrust pub
        self.thrust_pub.publish(msg)
        # Store time last message was published
        self.last_thrust_time = self.get_clock().now()

    def wheel_callback(self, msg):
        if self.kill_state:
            self.get_logger().info("Kill state is active. Ignoring wheel command.")
            return
        # Publish the message to wheel pub
        self.wheel_pub.publish(msg)
        # Store time last message was published
        self.last_wheel_time = self.get_clock().now()
    
    def send_zero_thrust(self):
        zero_thrust_msg = ThrusterCommand()
        zero_thrust_msg.header.stamp = self.get_clock().now().to_msg()
        zero_thrust_msg.duty_cycle = np.zeros(8)
        self.thrust_pub.publish(ThrusterCommand())
    
    def send_zero_wheel(self):
        zero_wheel_msg = WheelVelCommand()
        zero_wheel_msg.header.stamp = self.get_clock().now().to_msg()
        zero_wheel_msg.vel = [0.0]
        self.wheel_pub.publish(WheelVelCommand())

    def set_check_period(self, check_period):
        self.wait_period = check_period

def main(args=None):
    rclpy.init(args=args)

    safety_filter = SafetyFilter()

    rclpy.spin(safety_filter)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    safety_filter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()