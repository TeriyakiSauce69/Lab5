#!/usr/bin/env python3
import rospy
import math
import numpy as np
from robot_vision_lectures.msg import XYZarray
from robot_vision_lectures.msg import SphereParams


msg_received = False

xyz_msg = XYZarray()
sphere_param_msg = SphereParams()

xyz_array = []


def xyz_coord(data):

	global msg_received
	global xyz_array
	xyz_msg.points = data.points
	
	for x in data.points:
		xyz_array.append(x)
		
	print(type(xyz_array))



def sphere_param(data):
	global sphere_param_msg 
	xyz_msg.xc = data.xc
	xyz_msg.yc = data.yc
	xyz_msg.zc = data.zc
	xyz_msg.radius = data.radius
	




if __name__ == '__main__':
	# initialize the node
	rospy.init_node('sphere_fit', anonymous = True)
	# add a publisher for sending joint position commands
	
	#Subscribe to get the XYZ stuff
	rospy.Subscriber('/xyz_cropped_ball', XYZarray, xyz_coord)
	
	#
	pos_pub = rospy.Publisher('/sphere_params', SphereParams, queue_size = 1)
	
	
	# set a 10Hz frequency for this loop
	loop_rate = rospy.Rate(10)
	
	while not rospy.is_shutdown():
	
		
			
		loop_rate.sleep()
		
			
			
