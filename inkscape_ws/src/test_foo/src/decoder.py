#!/usr/bin/env python
# license removed for brevity
"""
ROS node that will decode encoder data from the robot into actual X,Y position.

@author Shashank Swaminathan
"""
import rospy
from std_msgs.msg import String
import math

class RaggedyROS():
    """
    Class to encapsulate the decoding work.
    """
    def __init__(self):
        """
        Init function. Initializes all relevant constants and arrays, as well as ROS Publishers and Subscribers.
        """
        self.prev_head = math.pi/4*0 # heading
        self.prev_xy = [0,0] # x | y
        self.prev_t = 0
        self.prev_lr = [0,0] # Left | Right

        self.abs_d = 0 # absolute distance
        self.ro = [0,0] # R | Theta
        self.vw = [0,0] # V | w
        self.head = math.pi/2*0 # heading
        self.xy = [0,0] # x | y
        self.t = 0
        self.lr = [0,0] # Left | Right
        self.init = 0

        self.d = 0.1031875
        self.circumference = 0.0635*math.pi
        self.reduction = 50

        rospy.init_node('curr_pos1', anonymous=True)
        self.pub = rospy.Publisher('/curr_pos/robot1', String, queue_size=10)
        rospy.Subscriber("/encoder_data/robot1", String, self.calculate_pos)

    def run(self):
        """
        Looping section of code. While not shutdown, continually publish data of the following form - [r, theta, x, y, heading, hasBeenInitialized?]
        """
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            pos = ','.join(list(map(str,[self.ro[0],
                                         self.ro[1],
                                         self.xy[0],
                                         self.xy[1],
                                         self.head,
                                         self.init])))
            self.pub.publish(pos)
            rate.sleep()

    def calculate_pos(self, msg):
        """
        Callback function that calculates the X,Y position of the robot based on encoder data. Only does so if the initialization has been completed. Running the callback function the first time completes the initialization.

        :param msg: ROS Message.
        """
        encoder_vals = list(map(float, msg.data.split(','))) # Parse out encoder data
        if self.init == 1:
            # Convert encoder data from pure wheel count data to mm traveled per wheel
            right_encoder = encoder_vals[1]*self.circumference/self.reduction
            left_encoder = encoder_vals[0]*self.circumference/self.reduction
            # rospy.loginfo("Encs: %s", [left_encoder, right_encoder])
            self.t = encoder_vals[2] # store time when encoder data was recorded

            # Determine velocity of wheels based on previous encoder data and time
            V_R = (right_encoder-self.prev_lr[1])/(self.t-self.prev_t)
            V_L = (left_encoder-self.prev_lr[0])/(self.t-self.prev_t)
            # rospy.loginfo("VD: %s", [V_L, V_R])

            # Store encoder data into previous value array
            self.prev_lr[1] = right_encoder
            self.prev_lr[0] = left_encoder

            # Determine lin and ang velocity of robot from wheel velocities
            self.vw[0] = (V_R + V_L)/2
            self.vw[1] = (V_R - V_L)/self.d
            # rospy.loginfo("VW: %s", [self.vw[0], self.vw[1]])

            # Calc absolute distance traveled from V, Omega
            self.abs_d = self.vw[0]*(self.t - self.prev_t)
            # Calc heading change from V, Omega
            self.head = (self.prev_head + self.vw[1]*(self.t - self.prev_t)) % (2*math.pi)
            # rospy.loginfo("DH: %s", [self.abs_d, self.head])
            # Calc new X,Y from previously calculated values
            self.xy[0] = round(self.prev_xy[0] + math.cos(self.head) * self.abs_d,10)
            self.xy[1] = round(self.prev_xy[1] + math.sin(self.head) * self.abs_d,10)

            # Calculate R, Theta from X,Y values
            self.ro[0] = math.sqrt(self.xy[0]**2 + self.xy[1]**2)
            self.ro[1] = math.atan2(self.xy[1], self.xy[0])

            # Update previous value arays
            self.prev_t = self.t
            self.prev_head = self.head
            self.prev_xy[0] = self.xy[0]
            self.prev_xy[1] = self.xy[1]

            # rospy.loginfo(rospy.get_caller_id() + "I heard %s", encoder_vals)
            # rospy.loginfo("RO: %s", [self.ro[0], self.ro[1]])
            rospy.loginfo("XY: %s", [self.xy[0], self.xy[1]])
        else:
            # Complete init
            self.prev_t = encoder_vals[2]
            self.init = 1

# Main method
if __name__ == '__main__':
    encs = RaggedyROS()
    encs.run()
