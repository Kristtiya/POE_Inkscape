#!/usr/bin/env python
# license removed for brevity
import cv2
import numpy as np
from matplotlib import pyplot as plt
import rospy
from std_msgs.msg import String

class Planner():
    def __init__(self, filename):
        self.points = self.img_to_points(filename)
        self.img = cv2.imread(filename, 0)

    def img_to_points(self, filename):
        img = cv2.imread(filename, 0)
        img = cv2.bitwise_not(img)
        edges = self.get_edges(img)
        white_edges = self.get_white(img)
        pts = self.sort_points(white_edges)

        return pts

    def get_edges(self, img):
        edges = cv2.Canny(img, 100, 200)
        return edges

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

    def get_white(self, edges):
        white = []

        for i in range(0, len(edges)):
            for j in range(0, len(edges[i])):
                if edges[i][j] == 255:
                    white.append([i, j])

        return np.array(white)

    def sort_points(self, white):
        distances = np.apply_along_axis(distance, axis=1, arr=white)
        indices = np.arange(0, white.shape[0])
        combined = np.transpose(np.vstack((distances, indices)))
        sorted_pts = combined[combined[:, 0].argsort()]
        sorted_indeces = sorted_pts[:, 1].tolist()
        self.points = [to_polar(white[int(idx)]) for idx in sorted_indeces]

        return self.points

    def get_vectors(self):
        self.points.insert(0, np.asarray([0, 0]))
        vectors = []
        for i in range(1, len(self.points)-1):
            curr_pt = np.asarray(self.points[i])
            prev_pt = np.asarray(self.points[i-1])
            vector = curr_pt-prev_pt
            vectors.append(vector)

        return vectors

def to_meter(pixel):
    return 	0.0002645833*pixel


def distance(point):
    return np.sqrt(point[0]**2+point[1]**2)

def to_polar(point):
    x = point[0]
    y = point[1]

    r = to_meter(np.sqrt(x**2+y**2))
    theta = np.arctan2(y, x)

    return [r, theta]

def talker(points):
    pub = rospy.Publisher('des_pos', String, queue_size=10)
    rospy.init_node('planner', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    idx = 0
    while not rospy.is_shutdown() and idx < len(points):
        point_str = str(points[idx])
        rospy.loginfo(point_str)
        pub.publish(point_str)
        rate.sleep()

        idx += 1

if __name__ == '__main__':
    planner = Planner('/home/shreya/POE_Inkscape/inkscape_ws/src/test_foo/src/circle.png')
    points = planner.points

    # points = planner.points
    # for i in range(0, 20):
    #     print(points[i])
    #
    # vecs = planner.get_vectors()
    # for i in range(0, 20):
    #     print(vecs[i])
    try:
        talker(points)
    except rospy.ROSInterruptException:
        pass
