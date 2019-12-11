#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
import numpy as np

class Johnny:
    def __init__(self, r):
        self.john = [0,0,0,0,0,0]
        # self.pub_d = rospy.Publisher('/des_pos/robot1', String, queue_size=10)
        self.pub_d = rospy.Publisher('/motor_ctrls/robot1', String, queue_size=10)
        # self.pub_e = rospy.Publisher('/encoder_data/robot1', String, queue_size=10)
        # rospy.Subscriber("/curr_pos/robot1", String, self.cb)
        rospy.init_node('talker', anonymous=True)
        self.rate = rospy.Rate(r) # 10hz
    def run(self, k):
        # posx=np.cos(np.linspace(0,2*np.pi,100))*k
        # posy=np.sin(np.linspace(0,2*np.pi,100))*k
        # counter=x=y=0
        while not rospy.is_shutdown():
            # time=rospy.get_time()
            # if self.john[5] != 1:
            #     x=0
            #     y=0
            # else:
            #     x=posx[counter]
            #     y=posy[counter]
            #     counter = (counter + 1) % 100

            # # epos = ','.join(list(map(str,[x,y,time])))
            dpos = ','.join(list(map(str,[0,0,0])))
            rospy.loginfo(dpos)
            # self.pub_e.publish(epos)
            self.pub_d.publish(dpos)
            self.rate.sleep()

    def cb(self, msg):
        self.john = list(map(float, msg.data.split(',')))

if __name__ == '__main__':
    try:
        johnny = Johnny(0.7)
        johnny.run(4)
    except rospy.ROSInterruptException:
        pass
