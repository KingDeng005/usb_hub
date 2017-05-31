'''
THIS CLASS IS AN ENCAPSULATION OF USB HUB
'''

import numpy as np
import brainstem as bs
import rospy

class usb_hub(Object):

	def __init__(self, module_index=0):
		self.modules = [0x78ADDCC2, 0xD80C7D63]
		self.stem = bs.stem.USBStem()
		self.stem.connect(module_index)

	def connect(self, module_index):
		if self.stem.connect(self.modules[module_index]) is not 0:
			rospy.logerr('Usb hub %d connection failed...check your device' % module_index)
		else:
			rospy.loginfo('Usb hub %d connect success!' % module_index)
		self.usb = bs.stem.USB(self.stem)
	
	def enable(self, port_num):
		if port_num < 0 or port_num > 7:
			rospy.logerr('The port number is out of range...valid: 0-7')
		else:
			if self.usb.setPortEnable(port_num) is not 0:
				rospy.logerr('port %d enabling failed...might be usb hub connection problem' % port_num)
			else:
				rospy.loginfo('port %d enabling success!' % port_num)

	def disable(self, port_num):
		if port_num < 0 or port_num > 7:
			rospy.logerr('The port number is out of range...valid: 0-7')
		else:
			if self.usb.setPortDisable(port_num) is not 0:
				rospy.logerr('port %d disabling failed...might be usb hub connection problem' % port_num)
			else:
				rospy.loginfo('port %d disabling success!' % port_num)

	def enable_all(self):
		for port in range(8):
			self.enable(port)

	def disable_all(self):
		for port in range(8):
			self.disable(port)

	def get_state(self, port_num):
		bank = port_num / 4
		index = port_num % 4
		mask = 1 << (2*index)
		if self.usb.getHubState(bank) & mask is not 0:
			return 1    # port is working
		else:
			return 0    # port is not working

