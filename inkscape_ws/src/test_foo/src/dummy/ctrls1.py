#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

def talker():
    rospy.init_node('ctrls1', anonymous=True)
    pub = rospy.Publisher('/motor_ctrls/robot1', String, queue_size=10)
    rospy.Subscriber("/curr_pos/robot1", String, callback)
#    rospy.Subscriber("/des_pos/robot1", String, callback)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "2.0,2.0>"
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
