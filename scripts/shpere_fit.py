#!/usr/bin/env python3 
import rospy 
import math 
import numpy as np 
from robot_vision_lectures.msg import XYZarray 
from robot_vision_lectures.msg import SphereParams 
 
# Stuff we need 
sphere_param_msg = SphereParams() 
xyz_array = np.array([]) 
start = False 
 
 
def xyz_coord(data): 
    # Make it global so we can access outside 
    global xyz_array 
    global start 
 
    # Data we get back 
    xyz_array = data 
    start = True 
 
 
# Set values to SphereParam after doing filtering 
def sphere_param(filtered_x, filtered_y, filtered_z, filtered_rad): 
    global sphere_param_msg 
    sphere_param_msg.xc = filtered_x
 
    sphere_param_msg.yc = filtered_y
 
    sphere_param_msg.zc = filtered_z
 
    sphere_param_msg.radius = filtered_rad
 
 
def do_math(data): 
    A, B = [], [] 
 
    # Make A and B matrix, from in class example 
    for point in data.points: 
        A.append([2 * point.x, 2 * point.y, 2 * point.z, 1]) 
        B.append(point.x ** 2 + point.y ** 2 + point.z ** 2) 
 
    # Perform math, from in class example 
    P = np.linalg.lstsq(A, B, rcond=None)[0] 
 
    return P 
 
 
def noise_filter(xyzr,fil_gain,fil_out): 
	#Do math, for x,y,z, and r
    answer = fil_gain * xyzr + (1 - fil_gain) * fil_out 
    return answer 
 
 
if __name__ == '__main__': 
 
    # initialize the node 
    rospy.init_node('sphere_fit', anonymous=True) 
 
    # Subscribe to get the XYZ stuff 
    rospy.Subscriber('/xyz_cropped_ball', XYZarray, xyz_coord) 
 
    # Publish final results 
    pos_pub = rospy.Publisher('/sphere_params', SphereParams, queue_size=1) 
 
    # set a 10Hz frequency for this loop 
    loop_rate = rospy.Rate(10) 
 
 	#Intial values got from printing unflitered data from first iteration
    xfil_out = -0.014 
    yfil_out = -0.017 
    zfil_out = 0.48 
    rfil_out = 0.055 
 
 	#Very still
    fil_gain = 0.02 
    r_gain = 0.018 
 
    while not rospy.is_shutdown(): 
        # We get data we start 
        if start: 
            #Call function with data gotten from xyz_coord 
            
            #Get raw data
            P = do_math(xyz_array) 
            
            #Apply filter to each value, reassign so it will used in next iteration
            new_x = noise_filter( P[0], fil_gain, xfil_out)
            xfil_out = new_x 
            
            new_y = noise_filter(P[1], fil_gain, yfil_out) 
            yfil_out = new_y 
            
            new_z = noise_filter(P[2], fil_gain, zfil_out)
            zfil_out = new_z
              
            
            new_r = noise_filter(math.sqrt(P[0] ** 2 + P[1] ** 2 + P[2] ** 2 + P[3]), r_gain, rfil_out) 
            rfil_out = new_r 
 
 			#Send filtered values to be assigned
            sphere_param(new_x, new_y, new_z, new_r) 
            
            pos_pub.publish(sphere_param_msg) 
            
        loop_rate.sleep() 

 
