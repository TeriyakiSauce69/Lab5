#!/usr/bin/env python3
import rospy
import math
import numpy as np
from robot_vision_lectures.msg import XYZarray
from robot_vision_lectures.msg import SphereParams

#Stuff we need
sphere_param_msg = SphereParams()
xyz_array = np.array([])
start = False


def xyz_coord(data):
#Make it global so we can access outside
	global xyz_array
	global start
	
	#Data we get back
	xyz_array = data
	start = True
		
		
#Set values to SphereParam
def sphere_param(data):
	global sphere_param_msg 
	sphere_param_msg.xc = data[0]
	
	sphere_param_msg.yc = data[1]
	
	sphere_param_msg.zc = data[2]
	
	sphere_param_msg.radius = math.sqrt(data[0] ** 2 + data[1] ** 2 + data[2] ** 2 + data[3])

	
	
def do_math(data):
	A, B = [],[]
	
	#Make A and B matrix, from in class example
	for point in data.points:
		A.append([2*point.x, 2*point.y, 2*point.z, 1])
		B.append(point.x ** 2 + point.y ** 2 + point.z ** 2)
		
	#Perform math, from in class example
	P = np.linalg.lstsq(A, B, rcond=None)[0]
	
	#Call function to set appriate values to SphereParams
	sphere_param(P)
	
	return None
	
def noise_filter(P, Prev,first_time):
	if first_time:
		#Got these from printing unfiltered data from first iteration
		fil_gain = 0.03
		r_gain = 0.026
		P.xc = fil_gain * P.xc + (1 - fil_gain) * -0.014
		P.yc = fil_gain * P.yc + (1 - fil_gain) * -0.017	
		P.zc = fil_gain * P.zc + (1 - fil_gain) * 0.48
		P.radius = r_gain * P.radius + (1 - r_gain) * 0.055
		return P
		
	fil_gain = 0.03
	r_gain = 0.026
	
	P.xc = fil_gain * P.xc + (1 - fil_gain) * Prev.xc
	P.yc = fil_gain * P.yc + (1 - fil_gain) * Prev.yc	
	P.zc = fil_gain * P.zc + (1 - fil_gain) * Prev.zc
	P.radius = r_gain * P.radius + (1 - r_gain) * Prev.radius
		
	return P

if __name__ == '__main__':
	
	# initialize the node
	rospy.init_node('sphere_fit', anonymous = True)

	#Subscribe to get the XYZ stuff
	rospy.Subscriber('/xyz_cropped_ball', XYZarray, xyz_coord)
	
	#Publish final results
	pos_pub = rospy.Publisher('/sphere_params', SphereParams, queue_size = 1)
	
	# set a 10Hz frequency for this loop
	loop_rate = rospy.Rate(10)

	
	first_pass = True
	
	while not rospy.is_shutdown():
	#We get data we start 
		if start:	
			#Call function with data gotten from xyz_coord
			P = do_math(xyz_array)
		
			if first_pass == False:
				filt_sphere_param_msg = noise_filter(sphere_param_msg, filt_sphere_param_msg, False)
				#Publish data
				pos_pub.publish(filt_sphere_param_msg)
				
			
			if first_pass: 
				filt_sphere_param_msg = noise_filter(sphere_param_msg,sphere_param_msg, True )
				first_pass = False
				
		loop_rate.sleep()
		
			
			
