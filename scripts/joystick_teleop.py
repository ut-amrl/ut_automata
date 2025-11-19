#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

# disable ALSA audio driver for pygame
import os
os.environ['SDL_AUDIODRIVER'] = 'dsp'

import pygame
import time

from amrl_msgs.msg import AckermannCurvatureDriveMsg
from sensor_msgs.msg import Joy

import sys

# See the pygame docs for more joystick capabilities:
# https://www.pygame.org/docs/ref/joystick.html

INACTIVITY_RECONNECT_TIME = 5
RECONNECT_TIMEOUT = 2

class JoystickTeleop(Node):
    def __init__(self):
        super().__init__('joystick_teleop')

        # Initialize class variables
        self.steer_joystick = 0.0
        self.drive_joystick = 0.0
        self.is_enabled = False
        self.turbo_mode = False
        self.joystick_connected = False
        self.last_active_time = 0
        self.last_reconnect_try_time = 0
        self.joystick = None

        # Create publishers
        self.pub = self.create_publisher(AckermannCurvatureDriveMsg, 'ackermann_curvature_drive', 5)
        self.pub_joymsg = self.create_publisher(Joy, '/bluetooth_teleop/joy', 5)

        # Initialize joystick
        self.initJoystick()

        # Create timer for main loop (20Hz)
        self.timer = self.create_timer(0.05, self.timer_callback)

        self.last_active_time = time.time()
        self.last_reconnect_try_time = time.time()
        self.speed = 1.0
        self.turn = 0.25

    def checkConnectivity(self):
        now = time.time()
        if ((now - self.last_active_time > INACTIVITY_RECONNECT_TIME) and 
            (now - self.last_reconnect_try_time > RECONNECT_TIMEOUT)):
            self.last_reconnect_try_time = now
            print("Checking joystick connectivity.")
            pygame.joystick.quit()
            pygame.joystick.init()
            self.joystick_connected = False

        joystick_count = pygame.joystick.get_count()

        if (joystick_count < 1):
            self.joystick_connected = False
            print("****** Joystick disconnected ******\n")

        if ((not self.joystick_connected) and (joystick_count > 0)):
            self.joystick.quit()
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
            self.joystick_connected = True
            print("****** Joystick Reconnected ******\n")

    def readJoystick(self):
        pygame.event.pump()
        self.steer_joystick = -self.joystick.get_axis(0)
        self.drive_joystick = -self.joystick.get_axis(4) # 4 for xbox
        self.is_enabled = self.joystick.get_button(4) == 1
        self.turbo_mode = self.joystick.get_axis(2) >= 0.9 # 5 for xbox

        # Makes the joystick message
        joy_msg = Joy()
        joy_msg.header.stamp = self.get_clock().now().to_msg()
        joy_msg.header.frame_id = "base_link"

        for i in range(6): #7
            joy_msg.axes.append(self.joystick.get_axis(i))
            #if (self.joystick.get_axis(i)):
            #  self.last_active_time = time.time()
        
        # Get hat (d-pad)
        if self.joystick.get_numhats() > 0:
            hat = self.joystick.get_hat(0)
            joy_msg.axes.append(hat[0]) # Left/Right (Axis 6)
            joy_msg.axes.append(hat[1]) # Up/Down (Axis 7)
        else:
            joy_msg.axes.append(0.0)
            joy_msg.axes.append(0.0)
        for i in range(10): #10
            joy_msg.buttons.append(self.joystick.get_button(i))
            if (self.joystick.get_button(i)):
                self.last_active_time = time.time()
        self.pub_joymsg.publish(joy_msg)

    def initJoystick(self):
        pygame.init()

        # Initialize the joysticks
        pygame.joystick.init()
        # Get count of joysticks
        joystick_count = pygame.joystick.get_count()

        if joystick_count < 1:
            print('No joystick found!')
            print('Please connect a joystick and try again.')
            print('If your joystick is connected, ensure that the udev rules are set up correctly.')
            print('You can run the script "src/ut_automata/scripts/udev.sh" to set up the appropriate rule.')
            sys.exit(0)

        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        self.joystick_connected = True

        # Get the name from the OS for the controller/joystick
        name = self.joystick.get_name()
        print("Joystick name: {}".format(name) )

        if (self.joystick.get_numbuttons() < 5):
            print("Error: expected buttion[4] to be valid!")
            sys.exit(1)

        if (self.joystick.get_numaxes() < 4):
            print("Error: expected axis[0] and axis[3] to be valid!")
            sys.exit(1)

    def timer_callback(self):
        self.checkConnectivity()

        if self.joystick_connected:
            self.readJoystick()
            if self.turbo_mode:
                speed = 2.0
            else:
                speed = 1.0

            msg = AckermannCurvatureDriveMsg()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "base_link"

            print("Drive: {:.2f}% Steer: {:.2f}%  Enabled: {}".format(
                self.drive_joystick, self.steer_joystick, self.is_enabled))

            msg.velocity = self.drive_joystick * speed
            msg.curvature = self.steer_joystick * self.turn

            if self.is_enabled:
                self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    joystick_teleop = JoystickTeleop()

    try:
        rclpy.spin(joystick_teleop)
    except KeyboardInterrupt:
        pass
    finally:
        # Send stop message
        msg = AckermannCurvatureDriveMsg()
        msg.header.stamp = joystick_teleop.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.velocity = 0.0
        msg.curvature = 0.0
        joystick_teleop.pub.publish(msg)

        # Close joystick
        pygame.quit()

        joystick_teleop.destroy_node()
        rclpy.shutdown()

if __name__=="__main__":
    main()
