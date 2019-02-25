#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from config.configTopic import *

def callback(data):
	msg = rospy.get_caller_id() + "\n"
	msg += str(data)
	rospy.loginfo(msg)

def listener():
	rospy.loginfo("Connection...")
	rospy.init_node('GillouQuiEcoute', anonymous=False)
	rospy.loginfo("Connected !")
	rospy.Subscriber(topicName, Twist, callback)
	rospy.spin()

if __name__ == '__main__':
	listener()
