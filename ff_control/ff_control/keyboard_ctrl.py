import rclpy

import sys, select, os
if os.name == 'nt':
  import msvcrt, time
else:
  import tty, termios

class KeyboardController():
    def __init__(self, node_name="keyboard_ctrl_node"):
        super().__init__(node_name)
        if os.name != 'nt':
            settings = termios.tcgetattr(sys.stdin)

    def getKey():
        if os.name == 'nt':
            timeout = 0.1
            startTime = self.get_clock().now().to_msg()
            while(1):
                if msvcrt.kbhit():
                    if sys.version_info[0] >= 3:
                        return msvcrt.getch().decode()
                    else:
                        return msvcrt.getch()
                elif self.get_clock().now().to_msg() - startTime > timeout:
                    return ''

        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key