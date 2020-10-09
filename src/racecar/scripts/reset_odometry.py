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

reset_odometry = rospy.ServiceProxy('/zed/zed_node/reset_odometry', Empty)
reset_tracking = rospy.ServiceProxy('/zed/zed_node/reset_tracking', Empty)


def subscribe_data(data):
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
    print(x,y,euler[2])
    if(abs(x) < 0.05 and abs(y)<0.05 and abs(euler[2])<0.09):
        subprocess.call(["rosservice","call","/zed/zed_node/reset_tracking"])
        subprocess.call(["rosservice","call", "/zed/zed_node/reset_odometry"])
        rospy.logwarn("reset_odometry")




def listener():
    rospy.init_node('reset_odom_node', anonymous=True)
    #rospy.wait_for_service('/zed/zed_node/reset_odometry')
    #rospy.wait_for_service('/zed/zed_node/reset_tracking')
    rospy.Subscriber('pf/pose/odom', Odometry, subscribe_data)
    rospy.spin()


if __name__=="__main__":
    listener()
