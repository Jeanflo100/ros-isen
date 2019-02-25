#!/usr/bin/env python3

from __future__ import print_function

import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy

from geometry_msgs.msg import Twist

import sys, select, termios, tty

from config.configTopic import *

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >

t : up (+z)
b : down (-z)

anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit
"""

moveBindings = {
        'i':(1,0,0,0),
        'o':(1,0,0,1),
        'j':(0,0,0,-1),
        'l':(0,0,0,1),
        'u':(1,0,0,-1),
        ',':(-1,0,0,0),
        '.':(-1,0,0,-1),
        'm':(-1,0,0,1),
        'O':(1,1,0,0),
        'I':(1,0,0,0),
        'J':(0,-1,0,0),
        'L':(0,1,0,0),
        'U':(1,-1,0,0),
        '<':(-1,0,0,0),
        '>':(-1,1,0,0),
        'M':(-1,-1,0,0),
        't':(0,0,1,0),
        'b':(0,0,-1,0),
    }

speedBindings={
        'q':(1,1),
        'z':(-1,-1),
        'w':(1,0),
        'x':(-1,0),
        'e':(0,1),
        'c':(0,-1),
    }

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def vels(speed,turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    pub = rospy.Publisher(topicName, Twist, queue_size = 1)
    rospy.init_node('GillouQuiParle', anonymous=False)

    speedInit = rospy.get_param("-speed", 0.5)
    turnInit = rospy.get_param("-turn", 1.0)
    speed = speedInit
    turn = turnInit
    x = 0
    y = 0
    z = 0
    th = 0
    status = 0

    try:
        print(msg)
        print(vels(speed,turn))
        while(1):
            key = getKey()
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                z = moveBindings[key][2]
                th = moveBindings[key][3]
            elif key in speedBindings.keys():
		speed += speedInit * speedBindings[key][0]
		turn += turnInit * speedBindings[key][1]

		if (speed > turnInit * 10):
		    speed = turnInit * 10
		elif (speed < 0):
		    speed = 0
		if (turn < turnInit * 10):
		    turn = turnInit * 10
		elif (turn < 0):
		    turn = 0

                print(vels(speed,turn))
                if (status == 14):
                    print(msg)
                status = (status + 1) % 15
            else:
                x = 0
                y = 0
                z = 0
                th = 0
                if (key == '\x03'):
                    break

            twist = Twist()
            twist.linear.x = x*speed; twist.linear.y = y*speed; twist.linear.z = z*speed;
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th*turn
            pub.publish(twist)

    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        pub.publish(twist)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
