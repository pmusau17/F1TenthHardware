#! /usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from path_planning.msg import drive_param
import matplotlib.pyplot as plt
import matplotlib.animation as anim


# function returns the number of samples within half the width of the car to account for the car's size
# parameters: car_width
#             len: length to obstacle

def find_number_of_samples(car_width, len):
    angle = car_width / (2.0 * len)
    angle = np.arctan(angle)
    return int(np.ceil(angle / 0.0043633))


def callback(msg):
    arr = np.array(msg.ranges)
    plt.plot(arr)
    # pub = rospy.Publisher('/drive_parameters', drive_param, queue_size=10)
    # pub.publish(message)


rospy.init_node('scan_values')
sub = rospy.Subscriber('/scan', LaserScan, callback)
rospy.sleep(0.1)
rospy.spin()
