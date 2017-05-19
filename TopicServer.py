#!/usr/bin/env python

import rospy
from usb_hub_server.srv import *
from std_msgs.msg import String
import brainstem
import numpy as np
import time

# initialize brainstem module and connect to the device
spec = brainstem.discover.findFirstModule(brainstem.link.Spec.USB)
stem = brainstem.stem.USBStem()
stem.connect(0xD80C7D63)
usb = brainstem.stem.USB(stem, 0)

# get each channel (0-7) state  
index_0 = 1
index_8 = 0x100
index_16 = 0x10000
index_24 = 0x1000000

# serial number mapped to port number
camera_num = ["0x00000001",\
	      "0x00000002",\
	      "16369133",\
              "0x00000004",\
              "0x00000005",\
              "0x00000006",\
              "0x00000007",\
	      "0x00000008"]

# mask sequence for hub state
a = np.array([index_0, index_8, index_16, index_24])
index = np.vstack([a,a])
b = np.arange(4)
channel = np.vstack([b, b+4]) 

def err_handling(req):
	# locate which port
	port = camera_num.index(req.serial_num)
	usb.setPortDisable(port)	
	a = usb.setPortEnable(port)
	try_num = 5   # try 5 times to ensure enabling
	while a is not 0 and try_num >0:
		rospy.loginfo('usb_hub node: unable to enable the port...try again...')
		a = usb.setPortEnable(port)
		time.sleep(0.5)
		try_num -= 1	
	if a is 0:
		res_msg = "Device(serial number:%s) has been successfully enabled"%req.serial_num
		rospy.loginfo(res_msg) 
		res = req.serial_num + " 0"   # example return: 16369133 0 - enabling success	
		return err_stringResponse(res)
	else:	
		res_msg = "Device(serial number:%s) enabling failed"%req.serial_num
		rospy.logerr(res_msg)
		res = req.serial_num + " 1"  # example return: 16369133 1 - enabling fail
		return err_stringResponse(res)

def start_server():
	rospy.init_node('usb_hub')
	s = rospy.Service('usb_hub_server', err_string, err_handling)
	rospy.spin()

a = usb.setPortEnable(2)
start_server()
