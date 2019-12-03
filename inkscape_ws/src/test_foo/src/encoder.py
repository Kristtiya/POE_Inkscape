#!/usr/bin/env python
# license removed for brevity

## We need to add the following:
# Change the current usage of self.ro[1] to actual theta, and not heading
# Make a separate variable for heading
# Publish the true heading and (x,y) position
# i.e. -> /curr_pos/robot1 -> "$r,$theta,$x,$y,$heading"
# Update the way the path_planner parses the msg to match
import rospy
from std_msgs.msg import String
import math
from geometry_msgs.msg import Twist

class RicketyROS():
    def __init__(self):
        self.curr_vals = [0,0,0,0,0]
        self.des_vals = [0,0]
        self.e_L = 0
        self.e_R = 0

        self.d = 0.1031875
        self.circumference = 0.0635*math.pi
        self.reduction = 50

        rospy.init_node('des_pos1', anonymous=True)
        self.pub = rospy.Publisher('/motor_ctrls/robot1', String, queue_size=10)
        rospy.Subscriber("/curr_pos/robot1", String, self.store_curr_pos)
        rospy.Subscriber("/des_pos/robot1", String, self.det_new_pos)

    def run(self):
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            pos = ','.join(list(map(str,[self.e_L, self.e_R])))
            # pos = pos+'>'
            self.pub.publish(pos)
            rate.sleep()

    def store_curr_pos(self, msg):
        self.curr_vals = list(map(float, msg.data.split(',')))

    def det_new_pos(self, msg):
        self.des_vals = list(map(float, msg.data.split(',')))

        delta_dist = [self.des_vals[0]-self.curr_vals[2],
                      self.des_vals[1]-self.curr_vals[3]]
        new_head = math.atan2(delta_dist[1]/delta_dist[0])
        if new_head < 0:
            new_head = new_head + 2*math.pi
        delta_head = new_head - self.curr_vals[4]
        delta_abs_d = math.sqrt(delta_dist[0]**2 + delta_dist[1]**2)

        # Assume that we can reach there in 1 second (actual time amount irrelevant)
        v = delta_abs_d
        w = delta_head
        # Diff drive equations
        # Since we assume it happens in 1 second, also equals diff motion
        p_L = v - w*self.d/2
        p_R = v + w*self.d/2
        # Convert to encoder values
        self.e_L = p_L*self.reduction/self.circumference
        self.e_R = p_R*self.reduction/self.circumference

        rospy.loginfo(rospy.get_caller_id() + "I heard %s", encoder_vals)
        rospy.loginfo("I calculated %s", [e_L, e_R])

if __name__ == '__main__':
    new_encs = RicketyROS()
    new_encs.run()
