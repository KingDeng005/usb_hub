#!/usr/bin/env python

import sys
import rospy
from usb_hub_server.srv import *
import time
import brainstem

def start_client(sn):
	rospy.wait_for_service('usb_hub_server')
	try:
		reset = rospy.ServiceProxy('usb_hub_server',err_string)
		resp1 = reset(sn)
		return resp1.reset_msg
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e

def usage():
	return "%s [x, y]"%sys.argv[0]

spec = brainstem.discover.findFirstModule(brainstem.link.Spec.USB)
stem = brainstem.stem.USBStem()
stem.connect(0xD80C7D63)
usb = brainstem.stem.USB(stem, 1)

print "ready to shutdown .. "
time.sleep(3)
a = usb.setPortDisable(2)
print a
print "wait for a while .. "
time.sleep(3)
sn = "16369133"
print "serial number is: %s"%sn
print "The result is: %s "%(start_client(sn))
