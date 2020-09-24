#!/usr/bin/env python

import rospy
from racecar.msg import drive_param
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
import math
import numpy as np
from numpy import linalg as la
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import csv
import os
import rospkg 

class pure_pursuit:

    def __init__(self):

        self.LOOKAHEAD_DISTANCE = 0.30#1.70 # meters
        self.VELOCITY = 3.2 # m/s
        self.goal = 0
        self.read_waypoints()
        self.msg = drive_param()
        self.msg.velocity = 1.5#1.5

        # Publisher for 'drive_parameters' (speed and steering angle)
        self.pub = rospy.Publisher('drive_parameters', drive_param, queue_size=1)

	#Publisher for the goal point
	self.goal_pub = rospy.Publisher('/waypoint/goal', Point, queue_size=1)

        rospy.Subscriber("/zed/zed_node/pose", PoseStamped, self.callback, queue_size=1)

    # Import waypoints.csv into a list (path_points)
    def read_waypoints(self):


        #filename='/home/musaup/Documents/catkin_ws/src/f110-fall2018-skeletons/labs/wall_following/logs/pure-pursuit-wp-2019-04-07-22-39-51.csv'
        #filename='/home/musaup/Documents/catkin_ws/src/f110-fall2018-skeletons/labs/wall_following/logs/pure-pursuit-wp-2019-04-08-02-28-24.csv'
        #filename = os.path.join(dirname, '../waypoints/levine-waypoints.csv')

        
        # get an instance of RosPack with the default search paths
        rospack = rospkg.RosPack()
        #get the path for this paackage
        package_path=rospack.get_path('a_stars_pure_pursuit')
        filename=package_path+'/waypoints/waypoints_1.csv'

        with open(filename) as f:
            path_points = [tuple(line) for line in csv.reader(f)]

        # Turn path_points into a list of floats to eliminate the need for casts in the code below.
        self.path_points_x   = [float(point[0]) for point in path_points]
        self.path_points_y   = [float(point[1]) for point in path_points]
        self.path_points_w   = [float(point[2]) for point in path_points]

        #Initialize array of zeros with the same number of entries as the waypoint markers
        self.dist_arr= np.zeros(len(self.path_points_y))

   
    # Computes the Euclidean distance between two 2D points p1 and p2.
    def dist(self, p1, p2):
        return np.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

    # Input data is PoseStamped message from topic /pf/viz/inferred_pose.
    # Runs pure pursuit and publishes velocity and steering angle.
    def callback(self,data):

        qx=data.pose.orientation.x
        qy=data.pose.orientation.y
        qz=data.pose.orientation.z
        qw=data.pose.orientation.w

        quaternion = (qx,qy,qz,qw)
        euler   = euler_from_quaternion(quaternion)
        yaw     = euler[2] 

        x = data.pose.position.x
        y = data.pose.position.y

        self.path_points_x = np.array(self.path_points_x)
        self.path_points_y = np.array(self.path_points_y)

        ## finding the distance of each way point from the current position 

        for i in range(len(self.path_points_x)):
            self.dist_arr[i] = self.dist((self.path_points_x[i],self.path_points_y[i]),(x,y))

        ##finding those points which are less than the look ahead distance (will be behind and ahead of the vehicle)

        goal_arr = np.where((self.dist_arr < self.LOOKAHEAD_DISTANCE+0.3)&(self.dist_arr > self.LOOKAHEAD_DISTANCE-0.3))[0]

        ##finding the goal point which is the last in the set of points less than the lookahead distance
        ##if the closest points array could not be formed, then the point which is closest to the current position is the goal. 
        
        for idx in goal_arr:
            #line from the point position to the car position
            v1 = [self.path_points_x[idx]-x , self.path_points_y[idx]-y]
            #since the euler was specified in the order x,y,z the angle is wrt to x axis
            v2 = [np.cos(yaw), np.sin(yaw)]
            #find the angle between these two vectors NOTE:These are in world coordinates
            temp_angle = self.find_angle(v1,v2)
            if abs(temp_angle) < np.pi/2:
                self.goal = idx
                # print(self.goal)
                break

        ##finding the distance of the goal point from the vehicle coordinatesr

        L = self.dist_arr[self.goal]

        ##Transforming the goal point into the vehicle coordinate frame (This come straight from the paper)

        gvcx = self.path_points_x[self.goal] - x
        gvcy = self.path_points_y[self.goal] - y 
        goal_x_veh_coord = gvcx*np.cos(yaw) + gvcy*np.sin(yaw)
        goal_y_veh_coord = gvcy*np.cos(yaw) - gvcx*np.sin(yaw)

        # math: find the curvature and the angle 
        alpha = self.path_points_w[self.goal] - (yaw)
        k = 2 * math.sin(alpha)/L
        angle_i = math.atan(k*0.4)

        angle = angle_i*2
        angle = np.clip(angle, -0.4189, 0.4189) # 0.4189 radians = 24 degrees because car can only turn 24 degrees max
        print(angle)
        #self.set_speed(angle)
        self.const_speed(angle)

	#publish the goal in the vehicle coordinates. 
	goalPoint = Point(float(goal_x_veh_coord),float(goal_y_veh_coord),float(angle))
	self.goal_pub.publish(goalPoint)

        # print functions for DEBUGGING
       # print(self.path_points_x[self.goal],self.path_points_y[self.goal],self.path_points_w[self.goal])
       # print(x,y,180*yaw/math.pi)
       # print(goal_y_veh_coord,angle)
       # print(self.LOOKAHEAD_DISTANCE,self.msg.velocity)
       # print("*******")

    def send_command(self):
    	self.pub.publish(self.msg)

    # USE THIS FUNCTION IF CHANGEABLE SPEED IS NEEDED
    def set_speed(self,angle):
        if (abs(angle)>0.2018):
            self.LOOKAHEAD_DISTANCE = 1.2
            # self.msg.velocity = 1.5
            self.msg.angle = angle

            if self.msg.velocity - 1.5 >= 0.5:
                self.msg.velocity -= 0.5#0.7

        else:
            self.LOOKAHEAD_DISTANCE = 1.2
            # self.msg.velocity = 3.0
            self.msg.angle = angle

            if self.VELOCITY - self.msg.velocity > 0.2:
                self.msg.velocity += 0.2
        print(angle,self.msg.velocity)

    # USE THIS FUNCTION IF CONSTANT SPEED IS NEEDED
    def const_speed(self,angle):
        #self.LOOKAHEAD_DISTANCE = 2
        self.msg.angle = angle
        self.msg.velocity = 0.7#self.VELOCITY

    # find the angle bewtween two vectors    
    def find_angle(self, v1, v2):
        cosang = np.dot(v1, v2)
        sinang = la.norm(np.cross(v1, v2))
        return np.arctan2(sinang, cosang)


if __name__ == '__main__':
    rospy.init_node('pure_pursuit')
    C = pure_pursuit()  
    r = rospy.Rate(40)

    while not rospy.is_shutdown():
        #print(C.msg.angle," ",C.msg.velocity)
        C.send_command()
        r.sleep()
