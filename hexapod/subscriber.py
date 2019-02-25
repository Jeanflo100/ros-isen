#!/usr/bin/env python3

from config.configTopic import *
from config.configHexapod import *
import rospy
from geometry_msgs.msg import Twist


def callback(data):
	msg = rospy.get_caller_id() + "\n"
	msg += str(fonctionMapVectorT_H(data))
	rospy.loginfo(msg)


def listener():
	rospy.loginfo("Connection...")
	rospy.init_node('GillouQuiEcoute', anonymous=False)
	rospy.loginfo("Connected !")
	rospy.Subscriber(topicName, Twist, callback)
	rospy.spin()


if __name__ == '__main__':
	listener()
