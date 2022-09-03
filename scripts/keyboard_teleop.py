#!/usr/bin/env python
import roslib
roslib.load_manifest('ut_automata')
import rospy
from amrl_msgs.msg import AckermannCurvatureDriveMsg
from sensor_msgs.msg import Joy

import sys, select, termios, tty

banner = """
Moving around:
        w
   a    s    d
anything else : stop
CTRL-C to quit
"""

joystick_msg = Joy()
joystick_msg.header.seq = 0
joystick_msg.header.frame_id = "base_link"
joystick_msg.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
joystick_msg.buttons = [0, 0, 0, 0, 0, 0, 0, 0]
kThrottleAxis = 4
kSteeringAxis = 0
kManualButton = 4
kAutoButton = 5
kPersistentAutoButton = 7

keyBindings = {
  'w':(1,0),
  'd':(1,-1),
  'a':(1,1),
  's':(-1,0),
}

def getKey():
   tty.setraw(sys.stdin.fileno())
   select.select([sys.stdin], [], [], 0)
   key = sys.stdin.read(1)
   termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
   return key

speed = 1.0
turn = 1.0

def vels(speed,turn):
  return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
  print(banner)
  settings = termios.tcgetattr(sys.stdin)
  pub = rospy.Publisher('ackermann_curvature_drive',
                        AckermannCurvatureDriveMsg,
                        queue_size=5)
  joypub = rospy.Publisher('joystick', Joy, queue_size=5)
  rospy.init_node('keyop')

  x = 0
  th = 0
  status = 0

  try:
    while(1):
       joystick_msg.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
       joystick_msg.buttons = [0, 0, 0, 0, 0, 0, 0, 0]
       key = getKey()
       if key in keyBindings.keys():
          x = keyBindings[key][0]
          th = keyBindings[key][1]
          joystick_msg.buttons[kManualButton] = 1
       else:
          x = 0
          th = 0
          if (key == '\x03'):
             break
       if key == ' ':
          print('Persistent auto')
          joystick_msg.buttons[kPersistentAutoButton] = 1
       elif key == '.':
          print('Auto')
          joystick_msg.buttons[kAutoButton] = 1

       msg = AckermannCurvatureDriveMsg()
       msg.header.stamp = rospy.Time.now()
       msg.header.frame_id = "base_link"
       msg.velocity = x*speed
       msg.curvature = th*turn

       joystick_msg.axes[kThrottleAxis] = x
       joystick_msg.axes[kSteeringAxis] = th
       joystick_msg.header.stamp = rospy.Time.now()
       joypub.publish(joystick_msg)
      #  pub.publish(msg)

  except:
    print('error')

  finally:
    msg = AckermannCurvatureDriveMsg()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "base_link"
    msg.velocity = 0
    msg.curvature = 0
    pub.publish(msg)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
