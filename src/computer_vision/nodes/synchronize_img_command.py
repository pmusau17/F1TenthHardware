#!/usr/bin/env python
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from message_filters import ApproximateTimeSynchronizer, Subscriber
from ackermann_msgs.msg import AckermannDriveStamped
import imutils
from racecar.msg import drive_param
import os
import rospkg


class MessageSynchronizer:
    ''' Gathers messages with vehicle information that have similar time stamps
        /camera/zed/rgb/image_rect_color/compressed: 18 hz
        /camera/zed/rgb/image_rect_color: 18 hz
        /racecar/drive_parameters: 40 hz
    '''
    def __init__(self):
        self.image_rect_color_left=Subscriber('/zed/zed_node/left/image_rect_color',Image)
        self.image_rect_color_right=Subscriber('/zed/zed_node/right/image_rect_color',Image)
        self.ackermann_stamped=Subscriber('/ackermann_cmd_mux/input/teleop',AckermannDriveStamped)
        r = rospkg.RosPack()
        self.save_path_root=r.get_path('computer_vision')+'/data/'
        self.cv_bridge=CvBridge()
        self.count=0
        self.save_count=0
        self.straight_count =0 

        #create the time synchronizer
        self.sub = ApproximateTimeSynchronizer([self.image_rect_color_left,self.image_rect_color_right,self.ackermann_stamped], queue_size = 20, slop = 0.05)
        #register the callback to the synchronizer
        self.sub.registerCallback(self.master_callback)

    #callback for the synchronized messages
    #Note: a negative value means turning to the right, a postive value means turning to the left
    def master_callback(self,image_left,image_right,ackermann_msg): #drive_param):
        #convert rosmsg to cv image
        try:
            cv_image=self.cv_bridge.imgmsg_to_cv2(image_left,"bgr8")
            cv_image2=self.cv_bridge.imgmsg_to_cv2(image_right,"bgr8")
            self.count+=1
        except CvBridgeError as e:
            print(e)

        #print(cv_image.shape,(ackermann_msg.drive.steering_angle,self.label_image(ackermann_msg.drive.steering_angle)),self.count)

        #convert the steering command to a string to I can store it with the image name
        #for efficient data storage
        command='%.10f' % ackermann_msg.drive.steering_angle
        #replace the period with ~ so it's a valid filename
        command=command.replace('.','~')
        
        #save path 
        save_path=self.save_path_root+self.label_image(ackermann_msg.drive.steering_angle)+'/'+str(rospy.Time.now())+'~'+command+'.jpg'
        save_path2=self.save_path_root+self.label_image(ackermann_msg.drive.steering_angle)+'/'+str(rospy.Time.now())+'~'+command+'.jpg'
        print(self.label_image(ackermann_msg.drive.steering_angle))

        if(self.count % 3==0 ackermann_msg.drive.speed>0.03):
            self.save_image(cv_image,save_path)
            self.save_image(cv_image2,save_path2)
        
    #function that categorizes images into left, weak_left, straight, weak_right, right
    def label_image(self,steering_angle):
        if(steering_angle<-0.261799):
            return "right"
        elif(steering_angle>0.261799):
            return "left"
        elif(steering_angle<-0.0523599 and steering_angle>-0.261799):
            return "weak_right"
        elif(steering_angle>0.0523599 and steering_angle<0.261799):
            return "weak_left"
        else:
            return "straight"

    
    def save_image(self,image,path):
        dirPath = os.path.split(path)[0]
        if not os.path.exists(dirPath):
            os.makedirs(dirPath)
            print('does not exist')

        if not "straight" in dirPath:
            self.save_count+=1
            cv2.imwrite(path,image)
       	    print(self.save_count)
        else: 
            self.straight_count+=1
            if(self.straight_count%5 == 0):
                cv2.imwrite(path,image)
       	        print(self.save_count)


if __name__=='__main__':
    rospy.init_node('image_command_sync')
    # get the name of the vesc for the car
    # initialize the message filter
    mf=MessageSynchronizer()
    
    # spin so that we can receive messages
    rospy.spin()    
