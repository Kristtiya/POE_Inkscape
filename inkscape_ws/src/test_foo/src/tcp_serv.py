#!/usr/bin/env python

import socket
import rospy
from std_msgs.msg import String
import sys

# HOST = '192.168.33.215'  # Standard loopback interface address (localhost)
# PORT = 9090        # Port to listen on (non-privileged ports are > 1023)

# s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# s.bind((HOST, PORT))
# s.listen()
# while True:
#     conn, addr = s.accept()
#     with conn:
#         print('Connected by', addr)
#         data = conn.recv(1024)
#         print(repr(data))
#         if not data:
#             break
#         conn.sendall(b"1,0>");

class SockPuppet:
    ''' Snagged from some docs online
    https://docs.python.org/2/howto/sockets.html
    '''
    def __init__(self, sock=None):
        if sock is None:
            self.sock = socket.socket(
                socket.AF_INET, socket.SOCK_STREAM)
        else:
            self.sock = sock
        # self.sock.listen(1)
        self.msg_in = None

    def connect(self, host, port):
        self.sock.bind((host, port))
        self.sock.listen(1)

    def mysend(self, msg, conn):
        msglen = len(msg)
        totalsent = 0
        while totalsent < msglen:
            sent = conn.send(msg[totalsent:])
            if sent == 0:
                raise RuntimeError("Socket connection broken")
            totalsent = totalsent + sent

    def myreceive(self, conn):
        msg = ''
        recv = 'nope'
        done = False
        while not done:
            recv = conn.recv(1024)
            marker = recv.find('>')
            if recv == '':
                raise RuntimeError("Socket connection broken")
            if marker != -1:
                msg = msg + recv[:marker]
                done = True
            else:
                msg = msg + recv
        return msg

    def run(self, msg_out):
        conn, addr = self.sock.accept()
        try:
            self.msg_in = self.myreceive(conn)
            self.mysend(msg_out, conn)
        except RuntimeError as e:
            print(str(e))
            pass
        finally:
            conn.close()
            return self.msg_in

    def shutdown(self,num):
        self.sock.shutdown(num)

class RicketyROS:
    def __init__(self, sock=None):
        self.pup = SockPuppet()
        self.usingRos = True if rospy is not None else False
        # Initialize each message. These won't publish if unset.
        self.enc_des = "-1.0,0.0>"
        self.enc_curr = "0.0,0.0"

        if self.usingRos:
            rospy.init_node('repeater', anonymous=True)
            # Create a subscriber to the command topic for each value
            self.sub_enc_des = rospy.Subscriber('/motor_ctrls/robot1', String, self._cb, queue_size=1)

            # Create a publisher to the corresponding topic for each value
            self.pub_enc_curr = rospy.Publisher('/encoder_data/robot1', String, queue_size=1)

            self.r = rospy.Rate(10)

    def run(self, host, port):
        try:
            self.pup.connect(host, port)
            while not rospy.is_shutdown():
                self.enc_curr = self.pup.run(self.enc_des)
                self.pub_enc_curr.publish(self.enc_curr)
                self.r.sleep()
        except KeyboardInterrupt:
            print("Detected keyboard interrupt, exiting")
            self.pup.shutdown(1)

    def _cb(self, msg):
        self.enc_des = msg

if __name__ == '__main__':
    rar = RicketyROS()

    # Optionally grab inputs from CL
    host = '192.168.33.215'
    port = 9090
    sys.argv = rospy.myargv(argv=sys.argv)
    if len(sys.argv) >= 3:
        host_name  = sys.argv[1]
        port = int(sys.argv[2])

    rar.run(host, port)
