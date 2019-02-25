#!/usr/bin/env python3

import rospy,time,serial
#from std_msgs.msg import String
from geometry_msgs.msg import Twist
from config.configHexapod import *
from config._serial_frame import *
from config.configTopic import *

port = "/dev/ttyUSB0"
baud = 38400
ser = serial.Serial(port, baud, timeout=1)

serial_frame = serial_frame()


def checksum(serial_frame):
	sum = serial_frame.right_v_byte + serial_frame.right_h_byte + serial_frame.left_v_byte + serial_frame.left_h_byte + serial_frame.button_byte
	return(255-(sum % 256))


def callback(data):
	vector = fonctionMapVectorT_H(data)
	serial_frame.header_byte = 255
	serial_frame.right_v_byte = vector.angular.y
	serial_frame.right_h_byte = vector.angular.z
	serial_frame.left_v_byte = vector.linear.x
	serial_frame.left_h_byte = vector.linear.y
	serial.frame.button_byte = 16
	serial_frame.end_byte = 0
	serial_frame.checksum_byte = checksum(serial_frame)

	ser.write(bytearray([serial_frame.header_byte,
		serial_frame.right_v_byte, serial_frame.right_h_byte,
		serial_frame.left_v_byte, serial_frame.left_h_byte,
		serial_frame.button_byte, serial_frame.end_byte,
		serial_frame.checksum_byte]))
#    if data.data == "stop":
#        serial_frame.right_v_byte = 128
#        serial_frame.right_h_byte = 128
#        serial_frame.left_v_byte = 128
#        serial_frame.left_h_byte = 128
#        serial_frame.button_byte = 16
#        serial_frame.checksum_byte = checksum(serial_frame)
#    if data.data == "forward":
#        serial_frame.right_v_byte = 128
#        serial_frame.right_h_byte = 128
#        serial_frame.left_v_byte = 250
#        serial_frame.left_h_byte = 128
#        serial_frame.button_byte = 16
#        serial_frame.checksum_byte = checksum(serial_frame)
#    if data.data == "left":
#        serial_frame.right_v_byte = 128
#        serial_frame.right_h_byte = 128
#        serial_frame.left_v_byte = 128
#        serial_frame.left_h_byte = 10
#        serial_frame.button_byte = 16
#        serial_frame.checksum_byte = checksum(serial_frame)
#    if data.data == "right":
#        serial_frame.right_v_byte = 128
#        serial_frame.right_h_byte = 128
#        serial_frame.left_v_byte = 128
#        serial_frame.left_h_byte = 250
#        serial_frame.button_byte = 16
#        serial_frame.checksum_byte = checksum(serial_frame)
#    if data.data == "backward":
#        serial_frame.right_v_byte = 128
#        serial_frame.right_h_byte = 128
#        serial_frame.left_v_byte = 10
#        serial_frame.left_h_byte = 128
#        serial_frame.button_byte = 16
#        serial_frame.checksum_byte = checksum(serial_frame)
#    if data.data == "twist left":
#        serial_frame.right_v_byte = 128
#        serial_frame.right_h_byte = 10
#        serial_frame.left_v_byte = 128
#        serial_frame.left_h_byte = 128
#        serial_frame.button_byte = 16
#        serial_frame.checksum_byte = checksum(serial_frame)
#    if data.data == "twist right":
#        serial_frame.right_v_byte = 128
#        serial_frame.right_h_byte = 250
#        serial_frame.left_v_byte = 128
#        serial_frame.left_h_byte = 128
#        serial_frame.button_byte = 16
#        serial_frame.checksum_byte = checksum(serial_frame)

#    serial_frame.header_byte = 255
#    serial_frame.right_v_byte = 128
#    serial_frame.right_h_byte = 128
#    serial_frame.left_v_byte = 128
#    serial_frame.left_h_byte = 128
#    serial_frame.button_byte = 16
#    serial_frame.end_byte = 0
#    serial_frame.checksum_byte = checksum(serial_frame)
#    while not rospy.is_shutdown():
#        rospy.Subscriber("hexapod/serial_command", String, callback)
#        ser.write(bytearray([serial_frame.header_byte,
#            serial_frame.right_v_byte, serial_frame.right_h_byte,
#            serial_frame.left_v_byte, serial_frame.left_h_byte,
#            serial_frame.button_byte, serial_frame.end_byte,
#            serial_frame.checksum_byte]))
#        time.sleep(0.033)



def arbotix_node():
	rospy.loginfo("Connection...")
	rospy.init_node('Hexapod', anonymous=False)
	rospy.loginfo("Connected !")
	rospy.Subscriber(topicName, Twist, callback)
	rospy.spin()


if __name__ == '__main__':
    try:
        arbotix_node()
    except rospy.ROSInterruptException:
        pass
