#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Float32MultiArray

class State():
    def __init__(self):
        self.prev_r = 0
        self.prev_theta = 0
        self.prev_t = 0

        self.r = 0
        self.theta = 0
        self.t = 0

        self.d = 0.5

        rospy.init_node('curr_pos1', anonymous=True)
        self.pub = rospy.Publisher('/curr_pos/robot1', Float32MultiArray, queue_size=10)
        rospy.Subscriber("/encoder_data/robot1", Int32MultiArray, self.calculate_pos)

    def run(self):
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            pos = Float32MultiArray(data=[self.r, self.theta])
            self.pub.publish(pos)
            rate.sleep()

    def calculate_pos(self, msg):
        encoder_vals = msg.data
        right_encoder = encoder_vals[0]
        left_encoder = encoder_vals[1]
        self.t = encoder_vals[2]

        # differentiate right_encoder
        V_R = right_encoder/self.t
        V_L = left_encoder/self.t

        V = (V_R + V_L)/2
        w = (V_R - V_L)/self.d

        self.r = self.prev_r + V*(self.t - self.prev_t)
        self.theta = self.prev_theta + w*(self.t - self.prev_t)

        self.prev_t = self.t
        self.prev_r = self.r
        self.prev_theta = self.theta

        rospy.loginfo(rospy.get_caller_id() + "I heard %s", encoder_vals)
        rospy.loginfo("I calculated %s", [self.r, self.theta])

if __name__ == '__main__':
    robot = State()
    robot.run()
