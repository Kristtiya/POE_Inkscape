#!/usr/bin/env python
# license removed for brevity
import cv2
import numpy as np
from matplotlib import pyplot as plt
import rospy
from std_msgs.msg import String

def distance(point):
    return np.sqrt(point[0]**2+point[1]**2)

def get_white_edges(edges):
    white = []
    white_x = []
    white_y = []
    for i in range(0, len(edges)):
        for j in range(0, len(edges[i])):
            if edges[i][j] == 255:
                white_x.append(i)
                white_y.append(j)

    return np.array(white_x), np.array(white_y)

def sort_points(white):
    distances = np.apply_along_axis(distance, axis=1, arr=white)
    indices = np.arange(0, white.shape[0])
    combined = np.transpose(np.vstack((distances, indices)))
    sorted_pts = combined[combined[:, 0].argsort()]
    sorted_indeces = sorted_pts[:, 1].tolist()

    sorted_x = []
    sorted_y = []
    for idx in sorted_indeces:
        pt = white[int(idx)]
        sorted_x.append(pt[0])
        sorted_y.append(pt[1])

    return sorted_x, sorted_y

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

if __name__ == "__main__":

    # Read raw image
    raw_img = cv2.bitwise_not(cv2.imread('/home/shreya/POE_Inkscape/inkscape_ws/src/test_foo/src/circle.png', 0))
    # plt.imshow(raw_img, cmap='gray')
    # plt.title("Original Image")

    # Find edges using Canny edge detection
    canny_edges = cv2.Canny(raw_img, 100, 200)
    plt.imshow(canny_edges, cmap='gray')

    # Identify the x,y coordinates of the white edges/the edges
    # that the robot should actually drive around
    white_x, white_y = get_white_edges(canny_edges)
    plt.plot(white_x, white_y, '*')

    # Sort the points from closest to 0 to furthest from 0 and check
    # to make sure all the points still align with the original image
    white_pts = np.column_stack((white_x, white_y))
    sorted_x, sorted_y = sort_points(white_pts)
    plt.plot(sorted_x, sorted_y, 'o')

    plt.legend(["Edges", "Sorted"])
    plt.show()
