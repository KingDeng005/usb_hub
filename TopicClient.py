#!/usr/bin/env python

import sys
import rospy
#from usb_hub_server.srv import *
import time
import brainstem
import rosservice
from std_msgs.msg import String

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
stem.connect(2024660162)
usb = brainstem.stem.USB(stem, 0)

print "ready to shutdown .. "
time.sleep(3)
a = usb.setPortDisable(1)
stem.disconnect()
print a
print "wait for a while .. "
time.sleep(3)
sn = str(16456118)
print "serial number is: %s"%sn
print type(rosservice.call_service('/usb_hub_server',[sn])[1])
print "The result is: %s "%(rosservice.call_service('/usb_hub_server',[sn])[1].reset_msg)
