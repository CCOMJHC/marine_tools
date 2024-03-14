#!/usr/bin/env python3


import rospy
import sys
from nmea_msgs.msg import Sentence

import socket

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def nmeaCallback(msg: Sentence):
  print(msg)
  s.sendto(bytes(msg.sentence, 'utf-8'), (address, port))



rospy.init_node("nmea_to_udp", sys.argv)

imu_sub = rospy.Subscriber('nmea', Sentence, nmeaCallback)

address = rospy.get_param('~address', '127.0.0.1')
port = rospy.get_param('~port', 4322)

rospy.spin()
