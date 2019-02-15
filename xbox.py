#!/usr/bin/env python
#pip install inputs
from inputs import get_gamepad
from geometry_msgs.msg import Twist
import rospy

topicTXmin = -10
topicTXmax = 10
topicTYmin = -10
topicTYmax = 10
topicTZmin = -10
topicTZmax = 10

topicRXmin = -10
topicRXmax = 10
topicRYmin = -10
topicRYmax = 10
topicRZmin = -10
topicRZmax = 10

newLZmin = 0
newLZmax = 100
newRZmin = 0
newRZmax = 100

topicMaximas = [[[topicTXmin, topicTXmax], [topicTYmin, topicTYmax], [topicTZmin, topicTZmax]], [[topicRXmin, topicRXmax], [topicRYmin, topicRYmax], [topicRZmin, topicRZmax]], [[newLZmin, newLZmax], [newRZmin, newRZmax]]]

topicName = '/cmd_vel'


posLXmin = -32768
posLXmax = 32767
posLYmin = 32767
posLYmax = -32768

posRXmin = -32768
posRXmax = 32767
posRYmin = 32767
posRYmax = -32768

posLZmin = 0
posLZmax = 1023
posRZmin = 0
posRZmax = 1023

xboxMaximas = [[[posLYmin, posLYmax], [posLXmin, posLXmax], []], [[], [posRYmin, posRYmax], [posRXmin, posRXmax]], [[posLZmin, posLZmax], [posRZmin, posRZmax]]]



def fonctionMap(x0, xMin0, xMax0, xMin1, xMax1):
	return xMin1 + (((x0 - xMin0) * (xMax1 - xMin1)) / (xMax0 - xMin0))


def fonctionMapX_T(valeur, index1, index2):
	newValeur = fonctionMap(valeur, xboxMaximas[index1][index2][0], xboxMaximas[index1][index2][1], topicMaximas[index1][index2][0], topicMaximas[index1][index2][1])
	moyenne = (topicMaximas[index1][index2][0] + topicMaximas[index1][index2][1])/2
	moyennePlus = moyenne + (topicMaximas[index1][index2][1] - moyenne)/5
	moyenneMoins = moyenne + (topicMaximas[index1][index2][0] - moyenne)/5
	if((newValeur > min(moyennePlus, moyenneMoins)) and (newValeur < max(moyennePlus, moyenneMoins))):
		newValeur = moyenne
	return newValeur


def move():
	codes = [["ABS_Y", "ABS_X", ""], ["", "ABS_RY", "ABS_RX"], ["ABS_Z", "ABS_RZ"]]
	rospy.init_node('Xbox', anonymous=True)
	pub = rospy.Publisher(topicName, Twist, queue_size=10)
	vel_msg = Twist()
	vel_msg.linear.x = 0
	vel_msg.linear.y = 0
	vel_msg.linear.z = 0
	vel_msg.angular.x = 0
	vel_msg.angular.y = 0
	vel_msg.angular.z = 0
	while not rospy.is_shutdown():
		events = get_gamepad()
		for event in events:
			if(event.state == 1):
				if(event.code == "BTN_SOUTH"):
					print "A"
					continue
				if(event.code == "BTN_EAST"):
					print "B"
					continue
				if(event.code == "BTN_NORTH"):
					print "X"
					continue
				if(event.code == "BTN_WEST"):
					print "Y"
					continue
				if(event.code == "ABS_HAT0X"):
					print "Droite"
					continue
				if(event.code == "ABS_HAT0Y"):
					print "Bas"
					continue
				if(event.code == "BTN_SELECT"):
					print "Select"
					continue
				if(event.code == "BTN_START"):
					print "Start"
					continue
				if(event.code == "BTN_TL"):
					print "L1"
					continue
				if(event.code == "BTN_TR"):
					print "R1"
					continue
			if(event.state == -1):
				if(event.code == "ABS_HAT0X"):
					print "Gauche"
					continue
				if(event.code == "ABS_HAT0Y"):
					print "Haut"
					continue
			if(event.state == 0):
				if(event.code == "BTN_THUMBL"):
					print "L3"
					continue
				if(event.code == "BTN_THUMBR"):
					print "R3"
					continue
			for i in range(len(codes)):
				for j in range(len(codes[i])):
					if(event.code == codes[i][j]):
						pos = fonctionMapX_T(event.state, i, j)
						if((i==0) and (j==0)):
							vel_msg.linear.x = pos
							continue
						if((i==0) and (j==1)):
							vel_msg.linear.y = pos
							continue
						if((i==0) and (j==2)):
							vel_msg.linear.z = pos
							continue
						if((i==1) and (j==0)):
							vel_msg.angular.x = pos
							continue
						if((i==1) and (j==1)):
							vel_msg.angular.y = pos
							continue
						if((i==1) and (j==2)):
							vel_msg.angular.z = pos
							continue
						if((i==2) and (j==0)):
							print "L2", pos
							continue
						if((i==2) and (j==1)):
							print "R2", pos
							continue
		pub.publish(vel_msg)
		print rospy.get_caller_id() + "\n" + str(vel_msg)

if __name__ == '__main__':
	try:
		move()
	except rospy.ROSInterruptException:
		pass

	#while(1):
		#events = get_gamepad()
		#for event in events:
			#print event.code, event.state, event.ev_type
