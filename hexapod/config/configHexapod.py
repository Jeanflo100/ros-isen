#!/usr/bin/env python3

from configTopic import *
from geometry_msgs.msg import Twist

hexapodTXmin = 0
hexapodTXmax = 256
hexapodTYmin = 0
hexapodTYmax = 256
hexapodTZmin = 0
hexapodTZmax = 256

hexapodRXmin = 0
hexapodRXmax = 256
hexapodRYmin = 0
hexapodRYmax = 256
hexapodRZmin = 0
hexapodRZmax = 256

def fonctionMap(x0, xMin0, xMax0, xMin1, xMax1):
	return xMin1 + (((x0 - xMin0) * (xMax1 - xMin1)) / (xMax0 - xMin0))

def fonctionMapVectorT_H(v):
	newV = Twist()

	newV.linear.x = fonctionMap(v.linear.x, topicTXmin, topicTXmax, hexapodTXmin, hexapodTXmax)
	newV.linear.y = fonctionMap(v.linear.y, topicTYmin, topicTYmax, hexapodTYmin, hexapodTYmax)
	newV.linear.z = fonctionMap(v.linear.z, topicTZmin, topicTZmax, hexapodTZmin, hexapodTZmax)
	newV.angular.x = fonctionMap(v.angular.x, topicRXmin, topicRXmax, hexapodRXmin, hexapodRXmax)
	newV.angular.y = fonctionMap(v.angular.y, topicRYmin, topicRYmax, hexapodRYmin, hexapodRYmax)
	newV.angular.z = fonctionMap(v.angular.z, topicRZmin, topicRZmax, hexapodRZmin, hexapodRZmax)

	return newV
