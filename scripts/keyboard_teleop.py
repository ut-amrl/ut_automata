#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from amrl_msgs.msg import AckermannCurvatureDriveMsg

import sys
import select
import termios
import tty

banner = """
Moving around:
        w
   a    s    d
anything else : stop
CTRL-C to quit
"""

keyBindings = {
    'w': (1, 0),
    'd': (1, -1),
    'a': (1, 1),
    's': (-1, 0),
}

class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyop')

        # Create publisher
        self.pub = self.create_publisher(AckermannCurvatureDriveMsg, 'ackermann_curvature_drive', 5)

        # Speed and turn parameters
        self.speed = 1.0
        self.turn = 1.0

        # Terminal settings
        self.settings = termios.tcgetattr(sys.stdin)

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def vels(self, speed, turn):
        return "currently:\tspeed %s\tturn %s " % (speed, turn)

    def publish_cmd_vel(self, x, th):
        msg = AckermannCurvatureDriveMsg()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.velocity = x * self.speed
        msg.curvature = th * self.turn
        self.pub.publish(msg)

    def run(self):
        print(banner)
        x = 0
        th = 0

        try:
            while rclpy.ok():
                key = self.getKey()
                if key in keyBindings.keys():
                    x = keyBindings[key][0]
                    th = keyBindings[key][1]
                else:
                    x = 0
                    th = 0
                    if (key == '\x03'):  # Ctrl-C
                        break

                self.publish_cmd_vel(x, th)

        except Exception as e:
            print(f'Error: {e}')

        finally:
            # Send stop command
            self.publish_cmd_vel(0, 0)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

def main(args=None):
    rclpy.init(args=args)

    keyboard_teleop = KeyboardTeleop()

    try:
        keyboard_teleop.run()
    except KeyboardInterrupt:
        pass
    finally:
        keyboard_teleop.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
