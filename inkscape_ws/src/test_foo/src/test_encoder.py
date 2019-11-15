#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Int32MultiArray

def talker():
    x = 1
    y = 1
    t = 0

    dx = 1
    dy = 2
    dt = 1
    rospy.init_node('encoder_data', anonymous=True)
    pub = rospy.Publisher('encoder_data/robot1', Int32MultiArray, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        msg_array = Int32MultiArray(data=[x, y, t])
        pub.publish(msg_array)
        rate.sleep()

        x += dx
        y += dy
        t += dt

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
