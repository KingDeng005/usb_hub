#!/usr/bin/env python
'''
THIS CLASS IS AN ENCAPSULATION OF USB HUB SERVER, ALSO AN INTERFACE BETWEEN ROS AND USB HUB SOFTWARE LAYER
CREATED BY FUHENG DENG 5/31/2017
'''

import rospy
from usb_hub_server.srv import *
from std_msgs.msg import String
import brainstem as bs
from USB_HUB import *
import numpy as np
import time

class usb_hub_server:

	def __init__(self):
		self.usb_hub = usb_hub()   # Only connect when necessary
		rospy.Service('usb_hub_server', err_string, self.err_handler)
	
	def err_handler(self,req):
		index = self.usb_hub.find_hub_and_port(req.serial_num)
		if not isinstance(index, tuple):
			rospy.logerr('Serial number doesn\'t exist in the list' % req.serial_num)
			return err_stringResponse(req.serial_num + " 1")
		# initialize the usb hub and get the usb instance
		hub_index = index[0]
		port_index = index[1] 	
		self.usb_hub.connect(hub_index)
		self.usb_hub.disable(port_index)	
		e_r = self.usb_hub.enable(port_index)
		try_num = 5   # try 5 times to ensure enabling
		while e_r is not 0 and try_num > 0:
			rospy.logwarn('usb_hub node: unable to enable the port...try again...')
			e_r = self.usb_hub.enable(port_index)
			time.sleep(0.2)
			try_num -= 1
		self.usb_hub.disconnect()
		# return based on enabling result
		if e_r is 0:
			rospy.loginfo('Device(serial number:%s) has been successfully enabled' % req.serial_num) 
			res = req.serial_num + " 0"   # example return: 16369133 0 - enabling success	
			return err_stringResponse(res)
		else:	
			rospy.logerr('Device(serial number:%s) enabling failed' % req.serial_num)
			res = req.serial_num + " 1"  # example return: 16369133 1 - enabling fail
			return err_stringResponse(res)
		

if __name__ == "__main__":
	rospy.init_node('USB_HUB_SERVER')
	_ = usb_hub_server()
	rospy.spin()
