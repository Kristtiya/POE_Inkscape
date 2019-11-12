#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

def talker():
    rospy.init_node('curr_pos1', anonymous=True)
    pub = rospy.Publisher('/curr_pos/robot1', String, queue_size=10)
    rospy.Subscriber("/encoder_data/robot1", String, callback)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
