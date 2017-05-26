#!/usr/bin/env python

import rospy
from usb_hub_server.srv import *
from std_msgs.msg import String
import brainstem
import numpy as np
import time

# serial number mapped to port number - 2 usb hubs
hub1_camera = ["0x00000001",\
	      	"0x00000002",\
	      	"16369133",\
              	"0x00000004",\
              	"0x00000005",\
              	"0x00000006",\
              	"0x00000007",\
	      	"0x00000008"]
hub2_camera = ["0x00000001",\
	      	"0x00000002",\
	      	"16369133",\
              	"0x00000004",\
              	"0x00000005",\
              	"0x00000006",\
              	"0x00000007",\
	      	"0x00000008"]
hub_camera = []
hub_camera.append(hub1_camera)
hub_camera.append(hub1_camera)
usb_hub_list = [0xD80C7D63, 0x0000000]  # decide later

def find_hub_and_port(camera_serial):
	err_cnt = 0
	for i in range(len(usb_hub_list)):
		try:
			port = hub_camera[i].index(camera_serial)
			return [i, port]
		except:
			continue
	return -1

def usb_hub_init(hub_serial):
	# initialize brainstem module and connect to the device
	spec = brainstem.discover.findFirstModule(brainstem.link.Spec.USB)
	stem = brainstem.stem.USBStem()
	res = stem.connect(hub_serial)
	if res is not 0:
		rospy.logerr("hub connection failed...")
	else:
		rospy.loginfo("hub connection successful...")
	usb = brainstem.stem.USB(stem, 0)
	return [usb, stem]

def usb_hub_terminate(usb, stem):
	stem.disconnect()
	usb = None

def err_handling(req):
	index = find_hub_and_port(req.serial_num)
	hub = index[0]
	port = index[1]
	if not isinstance(index, list):
		rospy.logerr('the serial number doesn\'t exist')
		res = req.serial_num + " 1"
		return err_stringResponse(res)
	# initialize the usb hub and get the usb instance
	[usb,stem] = usb_hub_init(index[0])
	usb.setPortDisable(index[1])	
	a = usb.setPortEnable(index[1])
	try_num = 5   # try 5 times to ensure enabling
	while a is not 0 and try_num > 0:
		rospy.logwarning('usb_hub node: unable to enable the port...try again...')
		a = usb.setPortEnable(port)
		time.sleep(0.2)
		try_num -= 1
	usb_hub_terminate(usb, stem)
	# see the enabling result
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
	rospy.init_node('TopicServer')
	s = rospy.Service('usb_hub_server', err_string, err_handling)
	rospy.spin()
if __name__ == "__main__":
	start_server()
