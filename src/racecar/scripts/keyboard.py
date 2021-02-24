#!/usr/bin/env python

import rospy
from racecar.msg import drive_param
from ackermann_msgs.msg import AckermannDriveStamped

import sys, select, termios, tty



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


speed = 0.7
turn = 0.5


if __name__=="__main__":

  settings = termios.tcgetattr(sys.stdin)
  rospy.init_node('keyboard', anonymous=True)
  args = rospy.myargv()[1:]
  if(len(args)==1):
    racecar_name = args[0]
  else:
    racecar_name = ''
  pub = rospy.Publisher(racecar_name+'/ackermann_cmd_mux/input/teleop', AckermannDriveStamped, queue_size=10)

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
       
       msg = drive_param()
       msg.velocity = x*speed
       msg.angle = th*turn
       rospy.loginfo(str(msg.velocity))
       rospy.loginfo(str(msg.angle))
       print(x*speed,th*turn)

       msg = AckermannDriveStamped();
       msg.header.stamp = rospy.Time.now();
       msg.header.frame_id = "base_link";

       msg.drive.speed = x*speed
       msg.drive.acceleration = 1
       msg.drive.jerk = 1
       msg.drive.steering_angle = th*turn
       msg.drive.steering_angle_velocity = 1

       pub.publish(msg)

  except:
    print 'error'

  finally:
    msg = drive_param()
    msg.velocity = 0
    msg.angle = 0

    msg = AckermannDriveStamped();
    msg.header.stamp = rospy.Time.now();
    msg.header.frame_id = "base_link";

    msg.drive.speed = x*speed
    msg.drive.acceleration = 1
    msg.drive.jerk = 1
    msg.drive.steering_angle = th*turn
    msg.drive.steering_angle_velocity = 1
    pub.publish(msg)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

