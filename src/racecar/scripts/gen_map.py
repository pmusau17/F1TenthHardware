#!/usr/bin/env python
import rospy
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap
import matplotlib.pyplot as plt
import rospkg 


import numpy as np

class GenerateMap:

    """Generates walls of maps:
       Note: -1: Unknown
              0: Free
              100: Occupied """
    def __init__(self,threshold=0.65,name="fgh"):
        rospy.wait_for_service('static_map')
        self.srv_proxy = rospy.ServiceProxy('static_map', GetMap)
        self.threshold = threshold
        self.points =[]
        self.x = []
        self.y = []
        self.x_free = []
        self.y_free = []
        self.name = name

        # use the rospack object to get paths
        rospack = rospkg.RosPack()
        package_path=rospack.get_path('racecar')
        # get the pid to create "unique" filenames
        self.filename=package_path+'/maps/{}_obstacles.txt'.format(self.name)
        self.filename2=package_path+'/maps/{}_freespace.txt'.format(self.name)


    def save_points(self):
        self.file = open(self.filename, 'w')
        self.file2 = open(self.filename2, 'w')
        for i in range(len(self.x)):
            xp = self.x[i] 
            yp = self.y[i]
            self.file.write('%f, %f\n' % (xp,yp))

        for i in range(0,len(self.x_free),100):
            xp = self.x_free[i] 
            yp = self.y_free[i]
            self.file2.write('%f, %f\n' % (xp,yp))

        self.file2.close()
        self.file.close()
        print("Done")
            

    def execute(self):
        msg = self.srv_proxy().map 

        # map metadata
        origin =[msg.info.origin.position.x+2.4,msg.info.origin.position.y-2.4]
        res = msg.info.resolution

        map_data= np.asarray(msg.data)
        grid_size=(msg.info.height,msg.info.width)
        map_data = map_data.reshape(grid_size)


        for i in range(map_data.shape[0]):
            for j in range(map_data.shape[1]):
                if(map_data[i][j]==100):
                    x_point = res*i + origin[0]
                    y_point = res*j + origin[1]
                    self.x.append(y_point)
                    self.y.append(x_point)
                    self.points.append((y_point,x_point))
                elif(map_data[i][j]==0):
                    x_point = res*i + origin[0]
                    y_point = res*j + origin[1]
                    pp = np.asarray([y_point,x_point])
                    dists = np.linalg.norm((self.points - pp),axis = -1)
                    if(dists.min()>0.5):
                        self.x_free.append(y_point)
                        self.y_free.append(x_point)
        self.save_points()
            
        
        plt.plot(self.x,self.y,'ro')
        plt.plot(self.x_free,self.y_free,'bo')
        plt.title('Plot of obstacles in track: {}'.format(self.name))
        plt.ylabel('y')
        plt.xlim([np.min(self.x)-1,np.max(self.x)+1])
        plt.ylim([np.min(self.y)-1,np.max(self.y)+1])
        #plt.ylim([-15,15])
        plt.xlabel('x')
        plt.show()


if __name__=="__main__":
    rospy.init_node("getMap")
    gm = GenerateMap()
    gm.execute()