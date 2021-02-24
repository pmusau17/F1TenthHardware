#!/usr/bin/env python
import rospy
import numpy as np
import atexit
import tf
from os.path import expanduser
from time import gmtime, strftime
from numpy import linalg as LA
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
import subprocess
import os 


# the null device is a device file that discards all data written to it but reports that the 
# write operation succeeded
devnull = open(os.devnull,'w')
start = 0
racecar_name = ''
def subscribe_data(data):
    global start
    quaternion = np.array([data.pose.pose.orientation.x, 
                           data.pose.pose.orientation.y, 
                           data.pose.pose.orientation.z, 
                           data.pose.pose.orientation.w])

    euler = tf.transformations.euler_from_quaternion(quaternion)
    speed = LA.norm(np.array([data.twist.twist.linear.x, 
                              data.twist.twist.linear.y, 
                              data.twist.twist.linear.z]),2)
    
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    now_time = rospy.Time.now()
    duration = (now_time - start).to_sec() 
    #print(x,y,euler[2],duration)
    if(abs(x) < 0.09 and abs(y)<0.09 and abs(euler[2])<0.15 and duration>5):
        rospy.logwarn("reset_odometry")
        subprocess.Popen(["rosservice","call", racecar_name+"/zed/zed_node/reset_odometry"],stdout=devnull,stderr=devnull)
        subprocess.Popen(["rosservice","call", racecar_name+"/zed/zed_node/reset_tracking"],stdout=devnull,stderr=devnull)
        start = rospy.Time.now()



def listener():
    rospy.Subscriber(racecar_name+'/pf/pose/odom', Odometry, subscribe_data)
    rospy.spin()


if __name__=="__main__":
    rospy.init_node('reset_odom_node', anonymous=True)
    args = rospy.myargv()[1:]
    if(len(args)==1):
       racecar_name = "/"+args[0]
    else:
       racecar_name = ''
    start = rospy.Time.now() 
    listener()
