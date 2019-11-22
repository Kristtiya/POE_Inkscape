#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
import math
from geometry_msgs.msg import Twist

class RaggedyROS():
    def __init__(self):
        self.prev_xy = [0,0] # x | y
        self.prev_ro = [0,math.pi/2] # R | Theta
        self.prev_t = 0
        self.prev_lr = [0,0] # Left | Right

        self.abs_d = 0
        self.xy = [0,0] # x | y
        self.ro = [0,math.pi/2] # R | Theta
        self.vw = [0,0] # V | w
        self.t = 0
        self.lr = [0,0] # Left | Right

        self.d = 0.1031875
        self.circumference = 0.0635*math.pi
        self.reduction = 50

        rospy.init_node('curr_pos1', anonymous=True)
        self.pub = rospy.Publisher('/curr_pos/robot1', String, queue_size=10)
        rospy.Subscriber("/encoder_data/robot1", String, self.calculate_pos)

    def run(self):
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            pos = ','.join(list(map(str,[self.ro[0], self.ro[1]])))
            self.pub.publish(pos)
            rate.sleep()

    def calculate_pos(self, msg):
        encoder_vals = list(map(float, msg.data.split(',')))
        right_encoder = encoder_vals[0]*self.circumference/self.reduction
        left_encoder = encoder_vals[1]*self.circumference/self.reduction
        self.t = encoder_vals[2]

        # differentiate right_encoder
        V_R = (right_encoder-self.prev_lr[1])/(self.t-self.prev_t)
        V_L = (left_encoder-self.prev_lr[0])/(self.t-self.prev_t)

        self.prev_lr[1] = right_encoder
        self.prev_lr[0] = left_encoder

        self.vw[0] = (V_R + V_L)/2
        self.vw[1] = (V_R - V_L)/self.d

        self.abs_d = self.vw[0]*(self.t - self.prev_t)
        self.ro[1] = (self.prev_ro[1] + self.vw[1]*(self.t - self.prev_t)) % (2*math.pi)
        self.xy[0] = self.prev_xy[0] + math.cos(self.ro[1]) * self.abs_d
        self.xy[1] = self.prev_xy[1] + math.sin(self.ro[1]) * self.abs_d

        self.ro[0] = math.sqrt(self.xy[0]**2 + self.xy[1]**2)

        self.prev_t = self.t
        self.prev_ro[0] = self.ro[0]
        self.prev_ro[1] = self.ro[1]
        self.prev_xy[0] = self.xy[0]
        self.prev_xy[1] = self.xy[1]

        rospy.loginfo(rospy.get_caller_id() + "I heard %s", encoder_vals)
        rospy.loginfo("I calculated %s", [self.ro[0], self.ro[1]])

if __name__ == '__main__':
    encs = RaggedyROS()
    encs.run()