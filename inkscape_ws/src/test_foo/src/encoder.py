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

class RallentandoROS():
    def __init__(self):
        self.curr_vals = [0,0,0,0,0]
        self.enc_vals = [0,0,0]
        self.des_vals = [0,0]
        self.e_L = 0
        self.e_R = 0

        self.d = 0.1031875
        self.circumference = 0.0635*math.pi
        self.reduction = 50

        rospy.init_node('des_pos1', anonymous=True)
        self.pub = rospy.Publisher('/motor_ctrls/robot1', String, queue_size=10)
        rospy.Subscriber("/encoder_data/robot1", String, self.store_enc_vals)
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
        rospy.loginfo("CV: %s", [self.curr_vals[2], self.curr_vals[3]])

    def store_enc_vals(self, msg):
        self.enc_vals = list(map(float, msg.data.split(',')))
        rospy.loginfo("EV: %s", [self.enc_vals[0], self.enc_vals[1]])

    def det_new_pos(self, msg):
        self.des_vals = list(map(float, msg.data.split(',')))

        delta_dist = [self.des_vals[0]-self.curr_vals[2],
                      self.des_vals[1]-self.curr_vals[3]]
        rospy.loginfo("DD: %s", [delta_dist[0], delta_dist[1]])

        new_head = math.atan2(delta_dist[1], delta_dist[0])
        if new_head < 0:
            new_head = new_head + 2*math.pi
        # rospy.loginfo("H: %s", [new_head])
        if new_head == 0 and delta_dist[0] == 0:
            delta_head = 0
        else:
            delta_head = new_head - self.curr_vals[4]
        delta_head, flip = self._headfind(delta_head)
        # rospy.loginfo("DH: %s", [delta_head])
        delta_abs_d = math.sqrt(delta_dist[0]**2 + delta_dist[1]**2)

        # Assume that we can reach there in 1 second (actual time amount irrelevant)
        v = delta_abs_d * flip
        w = delta_head
        # Diff drive equations
        # Since we assume it happens in 1 second, also equals diff motion
        p_L = v - w*self.d/2
        p_R = v + w*self.d/2
        # rospy.loginfo("PV: %s", [p_L, p_R])
        # Convert to encoder values
        self.e_L = p_L*self.reduction/self.circumference+self.enc_vals[0]
        self.e_R = p_R*self.reduction/self.circumference+self.enc_vals[1]

        rospy.loginfo("TE: %s", [self.e_L, self.e_R])
        # rospy.loginfo("-----")

    def _headfind(self, dh):
        a=abs(dh)
        s=math.copysign(1,dh)
        f=1
        if a < math.pi/2:
            return s*a, f
        if a > math.pi:
            a = math.pi*2 - a
            s = s * -1
            if a < math.pi/2:
                return s*a, f
        a = math.pi - a
        s = s * -1
        f = f * -1
        return s*a, f

if __name__ == '__main__':
    new_encs = RallentandoROS()
    new_encs.run()
