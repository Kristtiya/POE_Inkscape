#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Float32MultiArray
import math
from geometry_msgs.msg import Twist

class State():
    def __init__(self):
        self.prev_r = 0
        self.prev_theta = math.pi/2
        self.prev_t = 0
        self.prev_right = 0
        self.prev_left = 0

        self.r = 0
        self.theta = 0
        self.t = 0

        self.V = 0
        self.w = 0
        self.vel_msg = Twist()

        self.d = 0.1031875
        self.circumference = 0.0635*math.pi
        self.reduction = 50

        rospy.init_node('curr_pos1', anonymous=True)
        self.pub = rospy.Publisher('/curr_pos/robot1', Float32MultiArray, queue_size=10)
        self.turtle_pub = rospy.Publisher('/turtlesim1/turtle1/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber("/encoder_data/robot1", Int32MultiArray, self.calculate_pos)

    def run_turtle(self):
        self.vel_msg.angular.z = math.pi/2
        self.turtle_pub.publish(self.vel_msg)

        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            pos = Float32MultiArray(data=[self.r, self.theta])
            self.pub.publish(pos)

            self.vel_msg.linear.x = self.V*10
            # self.vel_msg.linear.z = self.V*math.sin(self.theta*180/math.pi)*10
            self.vel_msg.angular.z = self.w
            # self.vel_msg.angular.y = self.w
            # self.vel_msg.angular.z = 0
            self.turtle_pub.publish(self.vel_msg)

            rate.sleep()

    def run(self):
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            pos = Float32MultiArray(data=[self.r, self.theta])
            self.pub.publish(pos)
            rate.sleep()

    def calculate_pos(self, msg):
        encoder_vals = msg.data
        right_encoder = encoder_vals[0]*self.circumference/self.reduction
        left_encoder = encoder_vals[1]*self.circumference/self.reduction
        self.t = encoder_vals[2]

        # differentiate right_encoder
        V_R = (right_encoder-self.prev_right)/(self.t-self.prev_t)
        V_L = (left_encoder-self.prev_left)/(self.t-self.prev_t)

        self.prev_right = right_encoder
        self.prev_left = left_encoder

        self.V = (V_R + V_L)/2
        self.w = (V_R - V_L)/self.d

        self.r = self.prev_r + self.V*(self.t - self.prev_t)
        self.theta = self.prev_theta + self.w*(self.t - self.prev_t)

        self.prev_t = self.t
        self.prev_r = self.r
        self.prev_theta = self.theta

        rospy.loginfo(rospy.get_caller_id() + "I heard %s", encoder_vals)
        rospy.loginfo("I calculated %s", [self.r, self.theta])

if __name__ == '__main__':
    robot = State()
    robot.run_turtle()
