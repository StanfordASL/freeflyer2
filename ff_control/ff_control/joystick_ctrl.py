import rclpy

# Not sure we need a separate class here
class JoystickController():
    def __init__(self, node_name="joystick_ctrl_node"):
        super().__init__(node_name)
