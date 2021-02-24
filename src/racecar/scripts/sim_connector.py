#!/usr/bin/env python

import rospy
from racecar.msg import drive_param
from ackermann_msgs.msg import AckermannDriveStamped

import math



def vel_and_angle(data):
	
	msg = AckermannDriveStamped()
	msg.header.stamp = rospy.Time.now()
	msg.header.frame_id = "base_link"

	msg.drive.speed = data.velocity
	msg.drive.acceleration = 1
	msg.drive.jerk = 1
	msg.drive.steering_angle = data.angle
	msg.drive.steering_angle_velocity = 1

	pub.publish(msg)



	


if __name__=="__main__":
	rospy.init_node('sim_connect', anonymous=True)
        args = rospy.myargv()[1:]
        if(len(args)==1):
           racecar_name = args[0]
        else:
           racecar_name = ''
        pub = rospy.Publisher(racecar_name+'/ackermann_cmd_mux/input/teleop', AckermannDriveStamped, queue_size=5)
	rospy.Subscriber(racecar_name+'/drive_parameters', drive_param, vel_and_angle)
	rospy.spin()
