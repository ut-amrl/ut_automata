#!/usr/bin/env python
import roslib
roslib.load_manifest('ut_automata')
import rospy
from amrl_msgs.msg import AckermannCurvatureDriveMsg

import sys, select, termios, tty

banner = """
Moving around:
        w
   a    s    d
anything else : stop
CTRL-C to quit
"""

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
  rospy.init_node('keyop')

  x = 0
  th = 0
  status = 0

  try:
    while(1):
       key = getKey()
       if key in keyBindings.keys():
          x = keyBindings[key][0]
          th = keyBindings[key][1]
       else:
          x = 0
          th = 0
          if (key == '\x03'):
             break
       msg = AckermannCurvatureDriveMsg()
       msg.header.stamp = rospy.Time.now()
       msg.header.frame_id = "base_link"

       msg.velocity = x*speed
       msg.curvature = th*turn

       pub.publish(msg)

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
