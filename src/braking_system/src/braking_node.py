#! /usr/bin/env python
import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import message_filters
from std_msgs.msg import Float32
from braking_algorithm import BrakingAlgorithm

import rospy


def callback(laser, velocity):
    # the code below assumes a lidar with a 270 degree field of view
    # looks at the 90 degrees directly in front of the car
    num = len(laser.ranges) / 3
    lidar_ranges = np.asarray(laser.ranges[num: -num])
    distance_to_obstacle = np.min(lidar_ranges)
    vel = velocity.data * 1.2
    log_str = "true distance: ", distance_to_obstacle, " vel: ", vel
    distance_pub.publish(distance_to_obstacle)
    # rospy.loginfo(log_str)
    brakes = BrakingAlgorithm()

    input_arr = np.asarray([distance_to_obstacle / 12.0, vel / 4.0])
    input_arr.reshape(2, 1)
    braking_force = - brakes.algo(input_arr)

    log_str = "scaled dist: ", input_arr[0], " scaled vel: ", input_arr[1]
    # data logging
    scaled_dist_pub.publish(input_arr[0])
    scaled_vel_pub.publish(input_arr[1])
    # rospy.loginfo(log_str)

    log_str = "braking force: ", braking_force[0][0]
    # rospy.loginfo(log_str)
    acceleration_pub.publish(braking_force[0][0] * 5.2)


if __name__ == '__main__':
    rospy.init_node('braking_system')

    global distance_pub
    global acceleration_pub
    global scaled_dist_pub
    global scaled_vel_pub
    distance_pub = rospy.Publisher('obstacle_distance', Float32, queue_size=5)  # publish for data logging only
    scaled_dist_pub = rospy.Publisher('scaled_distance', Float32, queue_size=5)
    scaled_vel_pub = rospy.Publisher('scaled_velocity', Float32, queue_size=5)
    acceleration_pub = rospy.Publisher('acceleration', Float32, queue_size=5)

    laser_sub = message_filters.Subscriber('/scan', LaserScan)

    # more robust velocity readings when driving in a known environment
    # also tried reading ackermann_cmd_mux/output but it does not publish constantly
    # velocity_sub = message_filters.Subscriber('/pf/pose/odom', Odometry)
    vel_sub = message_filters.Subscriber('current_velocity', Float32)

    ts = message_filters.ApproximateTimeSynchronizer([laser_sub, vel_sub], queue_size=5, slop=0.1,
                                                     allow_headerless=True)
    ts.registerCallback(callback)
    rospy.spin()
