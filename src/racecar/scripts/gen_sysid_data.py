#!/usr/bin/env python
import rospy
from message_filters import ApproximateTimeSynchronizer, Subscriber
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np 
from racecar.msg import drive_param
import os
import rospkg
import atexit


class genSysIDData:
    ''' This class gather's data that will be used to perform greybox system identification of the F1Tenth Racecar in Matlab. 
        The model we will be using is the standard bicycle model that can be found here: (https://repository.upenn.edu/cgi/viewcontent.cgi?article=1908&context=cis_papers)

        The states of this model are [x,y,v,theta] where 
            x: x position 
            y: y position 
            v: linear velocity 
            theta: vehicle heading 

        It has two inputs: delta, u where: 
            delta: heading 
            u: throttle input (velocity setpoint in simulation)
    '''

    def __init__(self):
        r = rospkg.RosPack() 
        # The data will be stored in a csv file in the csv directory
        self.save_path_root=r.get_path('racecar')+'/sys_id/csv'
        self.odometry_sub=Subscriber('pf/pose/odom', Odometry)
        self.ackermann_stamped=Subscriber('/ackermann_cmd_mux/input/teleop',AckermannDriveStamped)
        self.sub = ApproximateTimeSynchronizer([self.odometry_sub,self.ackermann_stamped], queue_size = 20, slop = 0.020)

        self.campaign = 0
        self.filename=self.save_path_root+'{}_{}_{}.csv'.format("data",os.getpid(),self.campaign)
        self.file = open(self.filename, 'w+')

        #register the callback to the synchronizer
        self.sub.registerCallback(self.master_callback)
        self.startTime = 0
       
        self.count = 0 
        self.killatDiscontinuity = True
        self.previous_theta = None 


    #callback for the synchronized messages
    def master_callback(self,odom_msg,ackermann_msg): 
        

        # position 
        x = odom_msg.pose.pose.position.x
        y = odom_msg.pose.pose.position.y

        # Convert Quaternion to rpy
        rpy = euler_from_quaternion([odom_msg.pose.pose.orientation.x,
                                     odom_msg.pose.pose.orientation.y,
                                     odom_msg.pose.pose.orientation.z,
                                     odom_msg.pose.pose.orientation.w])

        # linear velocity 
        velx = odom_msg.twist.twist.linear.x
        vely = odom_msg.twist.twist.linear.y
        velz = odom_msg.twist.twist.linear.z



        # magnitude of velocity 
        speed = np.asarray([velx,vely])
        speed = np.linalg.norm(speed)

       

        # heading 
        delta = ackermann_msg.drive.steering_angle

        # throttle 

        u = ackermann_msg.drive.speed

        # time 
        if(self.count == 0):
            self.startTime = rospy.Time.now()
            time = 0 
        else:
            time = rospy.Time.now()- self.startTime
            time = np.round(time.to_sec(),decimals=2)
        
        if(self.count>2):
            if(abs(self.previous_theta - rpy[2])>0.3):
                rospy.logwarn("discontinuity, {}, {}".format(rpy[2],self.previous_theta))
                if(self.killatDiscontinuity):
                    self.reset()

     
        if(self.count>0):
            print("x:",x,"y:",y,"speed:",speed,"theta:",rpy[2],"u:",u,"delta:",delta,"time:",time)
            self.file.write('%f, %f, %f, %f, %f, %f, %f \n' % (time,x,y,speed,rpy[2],u,delta))
            self.previous_theta = rpy[2]

        self.count+=1
        
    def shutdown(self):
        self.file.close()
        print('Goodbye')
        

    def reset(self):
        self.previous_theta = None 
        self.count = -1
        self.file.close()
        self.campaign+=1
        self.filename=self.save_path_root+'{}_{}_{}.csv'.format("data",os.getpid(),self.campaign)
        self.file = open(self.filename, 'w+')


if __name__ == '__main__':
    rospy.init_node('sys_id')
    
    C = genSysIDData()
    atexit.register(C.shutdown)
    r = rospy.Rate(80)

    while not rospy.is_shutdown():
        r.sleep()
        

    
