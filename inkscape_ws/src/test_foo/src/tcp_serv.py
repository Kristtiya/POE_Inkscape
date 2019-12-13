#!/usr/bin/env python
"""
ROS node that runs a TCP server to connect to the ESP connected to the Arduino. Parses out received data, and publishes it to ROS Master.

@author Shashank Swaminathan
"""
import socket
import rospy
from std_msgs.msg import String
import sys

class SockPuppet:
    """
    Class to encapsulate TCP work. Snagged from some docs online: https://docs.python.org/2/howto/sockets.html.
    """
    def __init__(self, sock=None):
        """
        Init function. Creates INET socket.

        :param sock: Optional pre-prepared socket to use, instead of setting up one. If left None, the init function will create its own.
        """
        if sock is None:
            self.sock = socket.socket(
                socket.AF_INET, socket.SOCK_STREAM)
        else:
            self.sock = sock
        # self.sock.listen(1)
        self.msg_in = None

    def connect(self, host, port):
        """
        Binds socket to port.

        :param host: Host IP
        :param port: Port to bind to on host
        """
        self.sock.bind((host, port))
        self.sock.listen(1)

    def mysend(self, msg, conn):
        """
        Sends specified message to the client at the connection specified.

        :param msg: Message to send to client.
        :param conn: Connection object formed after connecting to client.
        """
        msglen = len(msg)
        totalsent = 0
        # print("I'm in a sending mode")
        while totalsent < msglen:
            # print("Entered sending")
            sent = conn.send(msg[totalsent:])
            if sent == 0:
                raise RuntimeError("Socket connection broken")
            totalsent = totalsent + sent
            # print("sent stuff")

    def myreceive(self, conn):
        """
        Receives data from the client. Does start and end marker checks to ensure that data hasn't been lost or corruped.

        :param conn: Connection object formed after connecting to client.
        """
        msg = ''
        recv = 'nope'
        done = False
        started = False
        while not done:
            recv = conn.recv(1024)
            smarker = recv.find('<')
            emarker = recv.find('>')
            print(recv)
            print(smarker)
            if recv == '':
                raise RuntimeError("Socket connection broken")
            if smarker != -1:
                started = True
            if started == True:
                if smarker != -1 and emarker != -1:
                    msg = msg + recv[smarker+1:emarker]
                    done = True
                elif emarker != -1:
                    msg = msg + recv[:emarker]
                    done = True
                    # print("Received stuff")
                else:
                    msg = msg + recv
        return msg

    def run(self, msg_out):
        """
        Main connection handler. Accepts connections from clients, receives data from client, and then sends response back to client.

        :param msg_out: Message to send to the client.

        :returns: Data received from the client.
        """
        conn, addr = self.sock.accept()
        print("Got connection from ", addr)
        try:
            self.msg_in = self.myreceive(conn)
            # print("Gonna send")
            self.mysend(msg_out, conn)
        except RuntimeError as e:
            print(str(e))
            pass
        except Exception as e:
            print(str(e))
            pass
        finally:
            conn.close()
            return self.msg_in

    def shutdown(self,num):
        """
        The server must end, like all good things.

        :param num: Shutdown flag.
        """
        self.sock.shutdown(num)

class RicketyROS:
    """
    Class handling ROS connection stuff for data received from sockets.
    """
    def __init__(self, sock=None):
        """
        Init function. Initializes internal buffers, and related ROS objects.

        :param sock: Optional pre-prepared socket to use, instead of setting up one. If left None, the init function will create its own.
        """
        self.pup = SockPuppet()
        self.usingRos = True if rospy is not None else False
        # Initialize each message. These won't publish if unset.
        self.enc_des = "0.0,0.0,1.0>"
        self.enc_curr = "0.0,0.0,0.0"

        if self.usingRos:
            rospy.init_node('repeater', anonymous=True)
            # Create a subscriber to the command topic for each value
            self.sub_enc_des = rospy.Subscriber('/motor_ctrls/robot1', String, self._cb, queue_size=1)

            # Create a publisher to the corresponding topic for each value
            self.pub_enc_curr = rospy.Publisher('/encoder_data/robot1', String, queue_size=1)

            self.r = rospy.Rate(10)

    def run(self, host, port):
        """
        Looping section of the code. While not shutdown, connect to robot and send/grab data. Afterwards, post received data to ROS master.

        :param host: Host IP
        :param port: Port to bind to on host.
        """
        try:
            self.pup.connect(host, port)
            while not rospy.is_shutdown():
                self.enc_curr = self.pup.run(self.enc_des)
                self.pub_enc_curr.publish(self.enc_curr)
                print("Des vals: ", self.enc_des)
                print("Curr vals: ", self.enc_curr)
                self.r.sleep()
        except KeyboardInterrupt:
            print("Detected keyboard interrupt, exiting")
            self.pup.shutdown(1)

    def _cb(self, msg):
        """
        Callback function to store data from ROS master to send to the robot.

        :param msg: ROS Message.
        """
        self.enc_des = msg.data

if __name__ == '__main__':
    rar = RicketyROS()

    # Optionally grab inputs from CL
    host = '192.168.35.49'
    port = 9090
    sys.argv = rospy.myargv(argv=sys.argv)
    if len(sys.argv) >= 3:
        host_name  = sys.argv[1]
        port = int(sys.argv[2])

    rar.run(host, port)
