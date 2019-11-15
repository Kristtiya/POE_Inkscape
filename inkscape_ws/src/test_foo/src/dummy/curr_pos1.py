#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

def talker():
    rospy.init_node('curr_pos1', anonymous=True)
    rospy.Subscriber("translator", Float32MultiArray, callback)
    rospy.spin()
    
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
