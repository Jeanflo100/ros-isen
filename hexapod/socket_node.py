#!/usr/bin/env python3

import rospy,socket,time
from std_msgs.msg import String

host = "192.168.0.115"
port = 5035
socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
socket.connect((host, port))
to_publish = String()
dirty = False

def socket_node():
    rospy.init_node('socket_node', anonymous = False)
    pub = rospy.Publisher('hexapod/serial_command', String, queue_size = 1)
    while not rospy.is_shutdown():
        received_message = socket.recv(128)
        if received_message == "Avancer\r\n":
            to_publish.data = "forward"
            dirty = True
	if received_message == "Position de base\r\n":
	    to_publish.data = "stop"
	    dirty = True
	if received_message == "Reculer\r\n":
	    to_publish.data = "backward"
	    dirty = True
	if received_message == "Tourne vers la droite\r\n":
	    to_publish.data = "twist right"
	    dirty = True
	if received_message == "Tourne vers la gauche\r\n":
	    to_publish.data = "twist left"
    	    dirty = True
	if received_message == "Droite crabe\r\n":
	    to_publish.data = "right"
	    dirty = True
	if received_message == "Gauche crabe\r\n":
	    to_publish.data = "left"
	    dirty = True
        if received_message == "Bouton A\r\n":
            to_publish.data = "forward"
            for i in range(0,100):
                pub.publish(to_publish)
		time.sleep(0.030)
            to_publish.data= "stop"
            for i in range(0,50):
                pub.publish(to_publish)
                time.sleep(0.030)
            to_publish.data = "twist left"
            for i in range(0,100):
                pub.publish(to_publish)
                time.sleep(0.030)
            to_publish.data= "stop"
            for i in range(0,50):
                pub.publish(to_publish)
                time.sleep(0.030)
            to_publish.data = "twist right"
            for i in range(0,100):
                pub.publish(to_publish)
                time.sleep(0.030)
            to_publish.data= "stop"
            for i in range(0,50):
                pub.publish(to_publish)
                time.sleep(0.030)
            to_publish.data = "backward"
            for i in range(0,50):
                pub.publish(to_publish)
                time.sleep(0.030)
            to_publish.data = "forward"
            for i in range(0,50):
                pub.publish(to_publish)
                time.sleep(0.030)
            dirty = True
	    to_publish.data = "stop"
        if dirty :
            pub.publish(to_publish)
            time.sleep(0.030)
    socket.close()

if __name__ == '__main__':
    try :
        socket_node()
    except rospy.ROSInterruptException :
        pass
