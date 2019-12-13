#!/usr/bin/env python
# license removed for brevity
"""
ROS Node meant to simulate the encoder movement of a ROS turtlesim robot. Only somewhat functional, and was never fully developed.

@author Shashank Swaminathan
"""
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
import numpy as np

class RaRaROS():
    def __init__(self):
        self.curr_time = 0
        self.prev_time = 0
        self.des_vals = [0,0]
        self.e_L = 0
        self.v_L = 0
        self.e_R = 0
        self.v_R = 0
        self.kp = 5

        self.d = 0.1031875
        self.circumference = 0.0635*np.pi
        self.reduction = 50

        rospy.init_node('turtle_encs', anonymous=True)
        self.turtle_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.enc_pub = rospy.Publisher("/encoder_data/robot1", String, queue_size=1)
        rospy.Subscriber("/motor_ctrls/robot1", String, self.store_des)

    def run(self):
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            self.curr_time = rospy.get_time()
            self.pid()
            self.e_L = self.e_L + (self.curr_time - self.prev_time) * self.v_L
            self.e_R = self.e_R + (self.curr_time - self.prev_time) * self.v_R
            # rospy.loginfo("EV: %s" % [self.e_L, self.e_R])
            enc_pos = ','.join(list(map(str,[self.e_L, self.e_R, self.curr_time])))
            self.enc_pub.publish(enc_pos)
            self.prev_time = self.curr_time
            rate.sleep()

    def store_des(self, msg):
        self.des_vals = list(map(float, msg.data.split(',')))
        # rospy.loginfo("DV: %s" % [self.des_vals[0], self.des_vals[1]])

    def pid(self):
        deltas=np.array([self.des_vals[0]-self.e_L,
                         self.des_vals[1]-self.e_R])

        self.v_L = deltas[0] * self.kp
        self.v_R = deltas[1] * self.kp

        v=(self.v_R + self.v_L)/2
        w=(self.v_R - self.v_L)/self.d

        v=v*self.circumference/self.reduction
        w=w*self.circumference/self.reduction

        msg=Twist(linear=Vector3(x=v,y=0,z=0),
                  angular=Vector3(x=0,y=0,z=w))

        self.turtle_pub.publish(msg)
        # rospy.loginfo("Deltas")
        # rospy.loginfo(deltas)
        # rospy.loginfo("msg output")
        # rospy.loginfo(msg)

if __name__ == '__main__':
    new_encs = RaRaROS()
    new_encs.run()
