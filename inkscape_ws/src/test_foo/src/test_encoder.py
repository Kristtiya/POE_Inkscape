#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String

def talker():
    right = 0
    left = 0
    t = 0

    dright = -5
    dleft = 5
    dt = 1

    rospy.init_node('encoder_data', anonymous=True)
    pub = rospy.Publisher('encoder_data/robot1', String, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        msg = ','.join(list(map(str,[right, left, t])))
        pub.publish(msg)
        rate.sleep()

        right += dright
        left += dleft
        t += dt

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
