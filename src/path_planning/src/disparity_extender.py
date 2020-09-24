#! /usr/bin/env python
import rospy
import numpy as np
import numpy.ma as ma
from sensor_msgs.msg import LaserScan
from path_planning.msg import drive_param
import matplotlib.pyplot as plt


def filter_array(arr, threshold):
    # trim end elements from array. Ranges outside of -90 to 90 degrees. This assumes a 270 degree lidar sweep
    num = int(len(arr) / 4)  # samples to remove from both sides
    trimmed_arr = arr[num: -num]
    SAFETY_FACTOR = 1  # distance to avoid collision between sides of the car and  walls
    # plt.subplot(1, 2, 1)
    # plt.plot(arr)

    i = 0
    while i < len(arr) - 1:
        if abs(arr[i + 1] - arr[i]) >= threshold:
            # overwrite the points after the disparity
            new_length = arr[i] if arr[i] < arr[i + 1] else arr[i + 1]

            margin = find_number_of_samples(0.3, new_length)  # number of samples around the safe zone
            # margin = 3
            #margin = 100
            # this code still includes some unreachable paths in the filtered array
            for j in range(0, margin):
                # the number of sample that need to be replaced
                if i + j < len(arr) and arr[i + j] > new_length:
                    arr[i + j] = new_length
            i = i + margin
        else:
            i = i + 1

    '''Matplotlib was used for debugging and testing'''
    # plt.subplot(1, 2, 2)
    # plt.plot(trimmed_arr)
    # plt.draw()
    # plt.pause(0.01)

    # find the angle corresponding to that distance
    # The sides of the car also have to be considered. If a minimum safe distance on the sides and back of car
    # is not found in all the samples in a direction the car is turning in, it will continue driving straight

    min = arr.min()
    min_index = -1
    if min <= SAFETY_FACTOR:
        min_index = arr.argmin()

    max_index = trimmed_arr.argmax()
    if max_index <= 360:
        if min_index > 360 or min_index == -1:
            angle = (max_index / -4) * (np.pi / 180)
        else:
            angle = 0  # continue driving in straight line
    else:
        if min_index < 720 or min_index == -1:
            angle = (max_index - 360) / 4 * (np.pi / 180)
        else:
            angle = 0  # continue driving straight

    print 'max distance at angle: ', angle * 180 / np.pi, ",index: ", max_index, ", distance: ", trimmed_arr[max_index]
    return trimmed_arr[max_index], angle


def find_velocity(distance):
    if distance < 0.5:
        return 0
    if distance >= 10:
        return 4
    else:
        return distance / 4


# function returns the number of samples within half the width of the car to account for the car's size
# parameters: car_width
#             len: length to obstacle

def find_number_of_samples(car_width, len):
    angle = car_width / (2.0 * len)
    angle = np.arctan(angle)
    return int(np.ceil(angle / 0.0043633))


def callback(msg):
    arr = np.array(msg.ranges)
    dist, angle = filter_array(arr, 0.2)
    vel = find_velocity(dist)
    if angle > np.pi / 4:
        angle = np.pi / 4
    if angle < -1 * np.pi / 4:
        angle = np.pi / 4 * -1

    message = drive_param(velocity=vel, angle=angle)
    pub = rospy.Publisher('/drive_parameters', drive_param, queue_size=10)
    pub.publish(message)


rospy.init_node('scan_values')
sub = rospy.Subscriber('/scan', LaserScan, callback)
rospy.sleep(0.1)
rospy.spin()
