#!/usr/bin/env python

'''
This file is used to transmit data from computer with camera to computer opperating baxter. 
This was required by the class to minimize risk to the robot. 
'''

import rospy
import time
import socket
import sys

from std_msgs.msg import String, Float32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3


sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
server_address = ('192.168.153.1', 10001)
message = 'This is the message.  It will be repeated.'

pub = rospy.Publisher('/my_Pos', Pose, queue_size=1)
    
p = Pose()
    
def run():
    
    i = 0
    while not rospy.is_shutdown():
        # Send data
        sent = sock.sendto(message, server_address)

        # Receive response
        data, server = sock.recvfrom(4096)
        val = data.split(',') 
        pos = [val[1],val[2],val[3]]
        q = [val[4],val[5],val[6],val[7]]
        
        
        p.position.x = float(val[1])
        p.position.y = float(val[2])
        p.position.z = float(val[3])
        # Make sure the quaternion is valid and normalized
        p.orientation.x = float(val[4])
        p.orientation.y = float(val[5])
        p.orientation.z = float(val[6])
        p.orientation.w = float(val[7])
        time.sleep(.01)
        i = i+1
        if i >  15:  #25:
            pub.publish(p)
            print(q)
            i = 1
    
    
    
if __name__ == '__main__':
    rospy.init_node('quinn_publisher', anonymous=True)
  
    while not rospy.is_shutdown():
        run()
    time.sleep(1)
