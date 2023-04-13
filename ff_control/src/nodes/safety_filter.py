import rclpy
from rclpy.node import Node
import time

from ff_msgs.msg import ThrusterCommand
from ff_msgs.msg import WheelVelCommand

class SafetyFilter(Node):

    def __init__(self):
        super().__init__('safety_filter')
        self.last_thrust_time = None
        self.last_wheel_time = None
        self.thrust_pub = self.create_publisher(ThrusterCommand, 'commands/duty_cycle_final', 10)
        self.wheel_pub = self.create_publisher(WheelVelCommand, 'commands/velocity_final', 10)
        self.thrust_sub = self.create_subscription(
            ThrusterCommand,
            'commands/duty_cycle',
            self.thrust_callback,
            10)
        self.wheel_sub = self.create_subscription(
            WheelVelCommand,
            'commands/velocity',
            self.wheel_callback,
            10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        zero_thrust_msg = ThrusterCommand()
        zero_thrust_msg.header.stamp = self.get_clock().now().to_msg()
        zero_thrust_msg.duty_cycle = [0.0] * 8

        # If no message was received in the last 30 seconds, publish a zero message
        if self.last_thrust_time is not None and time.time() - self.last_thrust_time > 30:
            self.thrust_pub.publish(ThrusterCommand())
            self.last_thrust_time = time.time()
        
        zero_wheel_msg = WheelVelCommand()
        zero_wheel_msg.header.stamp = self.get_clock().now().to_msg()
        zero_wheel_msg.vel = [0.0]

        if self.last_wheel_time is not None and time.time() - self.last_wheel_time > 30:
            self.wheel_pub.publish(WheelVelCommand())
            self.last_wheel_time = time.time()


    def thrust_callback(self, msg):
        # Publish the message to thrust pub
        self.thrust_pub.publish(msg)
        # Store time last message was published
        self.last_thrust_time = time.time()

    def wheel_callback(self, msg):
        # Publish the message to wheel pub
        self.wheel_pub.publish(msg)
        # Store time last message was published
        self.last_wheel_time = time.time()

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