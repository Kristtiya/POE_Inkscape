#!/usr/bin/env python
# license removed for brevity
import cv2
import numpy as np
from matplotlib import pyplot as plt
import rospy
from std_msgs.msg import Int32MultiArray
from connect import Connect

class Planner():
    def __init__(self, filename, resolution):
        # Initialize image stuff
        self.points = self._img_to_points(filename)
        self.img = cv2.imread(filename, 0)
        self.center = [self.img.shape[0]/2,self.img.shape[1]/2,0]
        self.scale = [1,1,1]
        self.xy = self._xy_transform()

        # Initialize ROS stuff
        self.curr_pos = [0,0]
        rospy.init_node('path_planner', anonymous=True)
        rospy.Subscriber("/curr_pos/robot1", String, self._cp_cb)
        self.state = [0,0] # Index of DP, Edge Line Number
        self.threshold = resolution
        self.pub = rospy.Publisher('/des_pos/robot1', String, queue_size=1)

    def run(self, rate):
        r = rospy.Rate(rate) # 10hz
        while not rospy.is_shutdown():
            des_pos = self.xy[state[0]]
            if self._rel_dist(self.curr_pos, des_pos) < self.threshold:
                self.state[0] = self.state[0] + 1
                if self.state[0] >= self.xy.shape[0]:
                    self.state[0] = self.state[0] - 1
                des_pos = self.xy[state[0]]
            if des_pos[2] == self.state[1]:
                new_point = 0
            else:
                new_point = 1
                self.state[1] = des_pos[2]
            pos = ','.join(list(map(str,[des_pos[0],des_pos[1],new_point])))
            self.pub.publish(pos)
            rate.sleep()

    def _img_to_points(self, filename):
        img = cv2.imread(filename, 0)
        img = cv2.bitwise_not(img)
        edges = cv2.Canny(img, 100, 200)
        conns = Connect(edges)
        return conns.grab_edges()

    def _xy_transform(self):
        xyn=self.points-self.center
        for i in range(xyn.shape[0]):
            for j in range(xyn.shape[1]-1):
                xyn[i][j]=xyn[i][j]*self.scale[j]
        return xyn

    def _rel_dist(self, cp, dp):
        return np.sqrt((cp[0]-dp[0])**2+(cp[1]-dp[1])**2)

    def _cp_cb(self, msg):
        cp = list(map(float, msg.data.split(',')))
        self.curr_pos = [cp[2], cp[3]]

    def display(self, edges, img):
        plt.subplot(121)
        plt.imshow(img, cmap='gray')
        plt.title("Original Image")
        plt.xticks([])
        plt.yticks([])

        plt.subplot(122)
        plt.imshow(edges, cmap='gray')
        plt.title("Edge Image")
        plt.xticks([])
        plt.yticks([])

        plt.show()

if __name__ == '__main__':
    filepath='/home/developer/POE_Inkscape/inkscape_ws/src/test_foo/src/circle.png'
    count = 0
    while (count < 3):
        try:
            planner = Planner(filepath, 5)
            planner.run(10)
        except rospy.ROSInterruptException:
            count = count + 1
            print("Failed to connect to ROS. Retrying ...")
    print("Exceeded failure count. Exiting.")
