#!/usr/bin/env python

import rospy
from usb_hub_server.srv import *
from std_msgs.msg import String
import brainstem
import numpy as np
import time

spec = brainstem.discover.findFirstModule(brainstem.link.Spec.USB)
stem = brainstem.stem.USBStem()
stem.connect(0xD80C7D63)
usb = brainstem.stem.USB(stem, 0)

# get each channel (0-7) state  
index_0 = 1
index_8 = 0x100
index_16 = 0x10000
index_24 = 0x1000000

# mask sequence for hub state
a = np.array([index_0, index_8, index_16, index_24])
index = np.vstack([a,a])
b = np.arange(4)
channel = np.vstack([b, b+4])

#low_level_node = rospy.Publisher('usb_hub_state',rospy.AnyMsg)
#rospy.init_node("low_level", anonymous=True)
#rate = rospy.Rate(10)

while not rospy.is_shutdown():
		s = ""
		for i in np.arange(2):
			for j in np.arange(4):
				s += "port %s:"%channel[i][j]
				if (usb.getHubState(i)).value & index[i][j]==0:
					s += "%s "%0				
					print bin(usb.getHubState(i).value)
					usb.setPortEnable(channel[i][j])
				else:
					s += "%s "%1
				s += "current is %s"%usb.getPortCurrent(channel[i][j])
		rospy.loginfo(s)
		time.sleep(1)
