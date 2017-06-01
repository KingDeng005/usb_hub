#!/usr/bin/env python
'''
THIS CLASS IS AN ENCAPSULATION OF USB HUB
CREATED BY FUHENG DENG 5/31/2017
'''

import numpy as np
import brainstem as bs
import rospy

# serial number mapped to port number - 2 usb hubs
hub0_device = ["16398896",\
	      	   "16456113",\
	      	   "16455921",\
               "16456119",\
               "16456117",\
               "0x0000000",\
               "0x0000000",\
	      	   "0x0000000"]
hub1_device = ["16456116",\
	      	   "16456118",\
	      	   "16455958",\
               "16455959",\
               "16456114",\
               "0x00000006",\
               "0x00000007",\
	      	   "0x00000008"]

class usb_hub:

	def __init__(self):
		self.modules = [0xD80C7D63,0x78ADDCC2]
		self.device_list = []
		self.device_list.append(hub0_device)  # camera 1 - 5
		self.device_list.append(hub1_device)  # camera 6 - 10
		self.stem = bs.stem.USBStem()
		self.usb = None

	def connect(self, module_index):
		if self.stem.connect(self.modules[module_index]) is not 0:
			rospy.logerr('Usb hub %d connection failed...check your device' % module_index)
		else:
			rospy.loginfo('Usb hub %d connect success!' % module_index)
		self.usb = bs.stem.USB(self.stem, 0)

	def disconnect(self):
		if self.stem.disconnect() is not 0:
			rospy.logerr('Usb hub disconnection failed...check your device')
		else:
			rospy.loginfo('Usb hub disconnect success!')
	
	def enable(self, port_num):
		if port_num < 0 or port_num > 7:
			rospy.logerr('The port number is out of range...valid: 0-7')
		else:
			if self.usb.setPortEnable(port_num) is not 0:
				rospy.logerr('port %d enabling failed...might be usb hub connection problem' % port_num)
				return 0
			else:
				rospy.loginfo('port %d enabling success!' % port_num)
				return 1

	def disable(self, port_num):
		if port_num < 0 or port_num > 7:
			rospy.logerr('The port number is out of range...valid: 0-7')
		else:
			if self.usb.setPortDisable(port_num) is not 0:
				rospy.logerr('port %d disabling failed...might be usb hub connection problem' % port_num)
				return 0
			else:
				rospy.loginfo('port %d disabling success!' % port_num)
				return 1

	def enable_all(self):
		for port in range(8):
			self.enable(port)

	def disable_all(self):
		for port in range(8):
			self.disable(port)

	def get_state(self, port_num):
		bank = port_num / 4  # bank 0: 0-3 bank 1: 4-7
		index = port_num % 4
		mask = 1 << (2*index)
		if self.usb.getHubState(bank) & mask is not 0:
			return 1    # port is working
		else:
			return 0    # port is not working

	def find_hub_and_port(self, camera_serial):
		for i in range(len(self.modules)):
			try:
				port = self.device_list[i].index(camera_serial)
				return (i, port)
			except:
				continue
		return None

