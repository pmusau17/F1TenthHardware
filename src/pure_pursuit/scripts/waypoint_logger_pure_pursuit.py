#!/usr/bin/env python
import rospy
import numpy as np
import atexit
import tf
import os 
from os.path import expanduser
from time import gmtime, strftime
from numpy import linalg as LA
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
import rospkg 



class WaypointLogger():

    def __init__(self,particle_filter=False):
        # use the rospack object to get paths
        rospack = rospkg.RosPack()
        self.particle_filter=particle_filter
        #get the path for this package
        package_path=rospack.get_path('pure_pursuit')
        # get the pid to create "unique" filenames
        self.filename=package_path+'/waypoints/record_{}.csv'.format(os.getpid())
        self.file = open(self.filename, 'w')

        self.waypoints=[[0,0]]

    def save_waypoint(self,data):
        pt = np.asarray([[data.pose.pose.position.x,data.pose.pose.position.y]])
        dist_arr = np.linalg.norm(np.asarray(self.waypoints)-pt,axis=-1)
        
        min_dist= np.min(dist_arr)
        if min_dist>0.05142135623730953:
            self.waypoints.append([data.pose.pose.position.x,data.pose.pose.position.y])
         
            print("x: {}, y: {}".format(data.pose.pose.position.x,data.pose.pose.position.y))
            self.file.write('%f, %f\n' % (data.pose.pose.position.x,
                                        data.pose.pose.position.y))

    def shutdown(self):
        self.file.close()
        print('Goodbye')
 
    def listener(self):
        if(not self.particle_filter):
            rospy.Subscriber('/zed/zed_node/camera_odom', Odometry, self.save_waypoint)
        else: 
            rospy.Subscriber('pf/pose/odom', Odometry, self.save_waypoint,queue_size=5)
        rospy.spin()

if __name__ == '__main__':
    # create Waypoint Object
    rospy.init_node('waypoints_logger', anonymous=True)
    args = rospy.myargv()[1:]
    if(len(args)>0):
    
        wp = WaypointLogger(particle_filter=True)
    else:
        wp = WaypointLogger()
    atexit.register(wp.shutdown)
    print('Saving waypoints...')
    try:
        wp.listener()
    except rospy.ROSInterruptException:
        pass
