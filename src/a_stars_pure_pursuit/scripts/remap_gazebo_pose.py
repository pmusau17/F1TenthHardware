#!/usr/bin/env python

import rospy
import numpy as np
import numpy.random as rand
import tf
from racecar.msg import drive_param
from geometry_msgs.msg import PoseStamped, Pose
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import GetModelState
from std_msgs.msg import Header
from nav_msgs.msg import Odometry

pub = rospy.Publisher("pf/viz/inferred_pose_fake", PoseStamped, queue_size=1)

racecar_pose = PoseStamped()

def timer_callback(event):
    global racecar_pose
    msg = PoseStamped()
    msg.pose = racecar_pose
    pub.publish(msg)

# Gets the racecar pose from gazebo/model_states topic. Since Gazebo has multiple
# models (racecar, ground plane) we have to index for the "racecar".
def robot_pose_update(data):
    racecar_pose.header.frame_id="map"
    racecar_pose.header.stamp=rospy.Time.now()
    racecar_pose.pose=data.pose.pose
    pub.publish(racecar_pose)
if __name__ == "__main__":
    rospy.init_node("remap_gazebo_pose")
    # Set the update rate
    #rospy.Timer(rospy.Duration(.025), timer_callback) # 40hz
    # Set subscribers
    rospy.Subscriber("/odom", Odometry, robot_pose_update)
    rospy.spin()
