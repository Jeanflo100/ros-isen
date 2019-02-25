#!/usr/bin/env python
# -*-coding:utf-8 -*

import rospy
from std_msgs.msg import String
from config.configTopic import *

def talker():
	pub = rospy.Publisher(topicName, String, queue_size=10)
	rospy.init_node('talker', anonymous=True)
	rate = rospy.Rate(10) # 10hz
	while not rospy.is_shutdown():
		hello_str = input("-> ")
		rospy.loginfo(hello_str)
		pub.publish(hello_str)
		rate.sleep()

if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass
