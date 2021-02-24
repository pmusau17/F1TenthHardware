#!/usr/bin/env python
import rospy
from racecar.msg import velocity_msg

class SpeedManager:

    #default is to publish speed commands of 1 m/s at 40 hz 
    def __init__(self,racecar_name,rate=40.0,speed=1.0):
        self.rate=rate
        self.speed=speed
        self.pub=rospy.Publisher(racecar_name+'/velocity_msg',velocity_msg,queue_size=5)

    def publish(self):
        r = rospy.Rate(self.rate) # 40hz
        while not rospy.is_shutdown():
            msg=velocity_msg()
            msg.header.stamp=rospy.Time.now()
            msg.velocity=self.speed
            self.pub.publish(msg)
            r.sleep()            

if __name__=="__main__":
    #get the arguments passed from the launch file   
    rospy.init_node('speed_manager', anonymous=True)
    args = rospy.myargv()[1:]
    if(len(args)>0):
       racecar_name = args[0]
    else:
       racecar_name = ''
    if(len(args)>1):
       speed = float(args[1]) 
    sp=SpeedManager(racecar_name,speed=speed)
    sp.publish()
    rospy.spin()
    

