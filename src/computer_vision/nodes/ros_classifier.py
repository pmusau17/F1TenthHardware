#!/usr/bin/env python
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from message_filters import ApproximateTimeSynchronizer, Subscriber
from cv_bridge import CvBridge, CvBridgeError
import numpy as np 
import imutils
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time

# import the custom message files defined the race package
from racecar.msg import drive_param
from racecar.msg import angle_msg

#import the tensorflow package
from tensorflow.python.keras.models import load_model
from tensorflow.python.keras.backend import clear_session

# import sys so we can use packages outside of this folder in
# either python 2 or python 3, I know it's janky, chill
import sys
import os
from pathlib import Path 
#insert parent directory into the path
sys.path.insert(0,str(Path(os.path.abspath(__file__)).parent.parent))
print(sys.path)

#import the preprocessing utils (helps with loading data, preprocessing)
from preprocessing.utils import ImageUtils

clear_session()

class ROS_Classify:

    #define the constructor 
    def __init__(self,model,decoupled=False,plot=False):


        self.cv_bridge=CvBridge()

        self.image_rect_color_left=Subscriber('/zed/zed_node/left/image_rect_color',Image)
        self.image_rect_color_right=Subscriber('/zed/zed_node/right/image_rect_color',Image)

        #create the time synchronizer
        self.sub = ApproximateTimeSynchronizer([self.image_rect_color_left,self.image_rect_color_right], queue_size = 50, slop = 0.05)

        # load the model 
        self.model=load_model(model)

        # keras predict function 
        self.model._make_predict_function()

        self.count = 0
        self.util=ImageUtils()


        #register the callback to the synchronizer
        self.sub.registerCallback(self.image_callback)

        # toggle plotting 
        self.plot = plot
        #depends how the model was trained
        try:
            self.height=self.model.layers[0].input_shape[1]
            self.width=self.model.layers[0].input_shape[2]
        except IndexError as e:
            self.height=self.model.layers[0].input_shape[0][1]
            self.width=self.model.layers[0].input_shape[0][2]

        self.classes=['left','right','straight','weak_left','weak_right']

        self.decoupled=decoupled
        if (not self.decoupled):
            self.pub=rospy.Publisher('drive_parameters', drive_param, queue_size=5)
        else:
            self.pub=rospy.Publisher('angle_msg',angle_msg,queue_size=5)
        

        if self.plot:
            #fields for plotting
            self.commands=[]
            self.times=[]
            self.start_time=time.time()
            #figure for live animation
            self.fig = plt.figure()
            self.ax = self.fig.add_subplot(1, 1, 1)
            self.window=4000
            #Animation
            # Set up plot to call animate() function periodically
            ani = animation.FuncAnimation(self.fig, self.animate, fargs=(self.commands, self.times), interval=1000)
            plt.show()
        

    #image callback
    def image_callback(self,image_left,image_right):
        #convert the ros_image to an openCV image
        try:
            orig_image=self.cv_bridge.imgmsg_to_cv2(image_left,"bgr8")/255.0
            cv_image=self.util.reshape_image(orig_image,self.height,self.width)
            cv_image2=self.cv_bridge.imgmsg_to_cv2(image_right,"bgr8")/255.0
            cv_image2=self.util.reshape_image(cv_image2,self.height,self.width)
            #print(cv_image.shape)
        except CvBridgeError as e:
            print(e)

        imgs = np.asarray([cv_image,cv_image2])
        #print(imgs.shape)

    
        pred=self.model.predict(imgs)
        self.count+=1

        pred1 = pred[0].argmax()
        pred2 = pred[1].argmax()

        pred = np.mean(pred,axis=0)

        print(self.classes[pred1],self.classes[pred2],self.classes[pred.argmax()])

        #publish the actuation command
        if(self.count>20):
            self.send_actuation_command(pred)


    #computes the actuation command to send to the car
    def send_actuation_command(self,pred):
        #get the label
        label=self.classes[pred.argmax()]
        if (label=="left"):
            angle=0.5108652353
        elif (label=="right"):
            angle=-0.5108652353
        elif (label=="straight"):
            angle=0.0
        elif (label=="weak_left"):
            angle=0.10179
        elif (label=="weak_right"):
            angle=-0.10179
        else: 
            print("error:",label)
            angle=0

        if (self.plot):
            self.commands.append(angle)
            self.times.append(time.time()-self.start_time)

        if (not self.decoupled):
            msg = drive_param()
            msg.header.stamp=rospy.Time.now()
            msg.angle = angle
            msg.velocity = 0.4
        else:
            msg=angle_msg()
            msg.header.stamp=rospy.Time.now()
            msg.steering_angle=angle
        self.pub.publish(msg)

    #function that animates the plotting
    def animate(self,i,commands,times):
        # Limit x and y lists to window items
        self.commands = self.commands[-self.window:]
        self.times = self.times[-self.window:]
        # Draw x and y lists
        self.ax.clear()
        self.ax.plot(self.times, self.commands)

        # Format plot
        plt.xticks(rotation=45, ha='right')
        plt.subplots_adjust(bottom=0.30)
        plt.title('Time (s) vs Steering Angle (radians)')
        plt.ylabel('Steering Angle (radians)')
        plt.xlabel('Time (s)')


if __name__=='__main__':
    rospy.init_node("classify_node",anonymous=True)
    #get the arguments passed from the launch file
    args = rospy.myargv()[1:]
    #get the racecar name so we know what to subscribe to
    #get the keras model
    model=args[0]


    #if there's more than two arguments then its decoupled
    if len(args)>2:
        il=ROS_Classify(model,decoupled=True)
    else:
        il=ROS_Classify(model)
    try: 
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting Down")
        cv2.destroyAllWindows()
