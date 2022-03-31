#!/usr/bin/env python
import numpy as np 
import math
from scipy.linalg import expm
# from lab4_header import *

"""
Use 'expm' for matrix exponential.
Angles are in radian, distance are in meters.
"""
def Get_MS():
	# =================== Your code starts here ====================#
	# Fill in the correct values for a1~6 and q1~6, as well as the M matrix
	M = np.array([[ 0,-1, 0, 390],
				 [ 0, 0,-1, 401],
				 [ 1, 0, 0, 215.5],
				 [ 0, 0, 0, 1]])
	
	# S-arrays 1-6
	S1 = np.array([[0,-1, 0, 150],
				  [1, 0, 0, 150],
				  [0, 0, 0,0],
				  [0, 0, 0, 0]])

	S2 = np.array([[0, 0, 1, -162],
				  [0, 0, 0, 0],
				  [-1, 0, 0, -150],
				  [0, 0, 0, 0]])

	S3 = np.array([[0, 0, 1, -162],
				  [0, 0, 0, 0],
				  [-1, 0, 0, 94],
				  [0, 0, 0, 0]])

	S4 = np.array([[0, 0, 1, -162],
				  [0, 0, 0, 0],
				  [-1, 0, 0, 307],
				  [0, 0, 0, 0]])

	S5 = np.array([[0, 0, 0, 0],
				  [0, 0, -1, 162],
				  [0, 1, 0, -260],
				  [0, 0, 0, 0]])

	S6 = np.array([[0, 0, 1, -162],
				  [0, 0, 0, 0],
				  [-1, 0, 0, 389],
				  [0, 0, 0, 0]])

	S = [S1, S2, S3, S4, S5, S6]

	# ==============================================================#
	return M,S


"""
Function that calculates encoder numbers for each motor
"""
def lab_fk(theta1, theta2, theta3, theta4, theta5, theta6):

	# 
	# Initialize the return_value
	return_value = [None, None, None, None, None, None]
	# =========== Implement joint angle to encoder expressions here ===========
	thetas = [theta1, theta2, theta3, theta4, theta5, theta6]
	M,S = Get_MS()

	i = 5
	while i >= 0:
		exp = S[i] * thetas[i]
		curr_e = expm(exp)
		M = curr_e.dot(M)
		i -= 1

	T = M
	# return T 
	print("Foward kinematics calculated:\n")

	print(str(T) + "\n")

	return_value[0] = theta1 + np.pi
	return_value[1] = theta2
	return_value[2] = theta3
	return_value[3] = theta4 - (0.5*np.pi)
	return_value[4] = theta5
	return_value[5] = theta6

	return return_value


"""
Function that calculates an elbow up Inverse Kinematic solution for the UR3
"""
def lab_invk(xWgrip, yWgrip, zWgrip, yaw_WgripDegree):
	# =================== Your code starts here ====================#
	x = xWgrip * 1000
	y = yWgrip * 1000
	z = zWgrip * 1000
	yaw = np.radians(yaw_WgripDegree)
	#===============================================================#
	# Find [x_cen, y_cen, z_cen]: 
	#===============================================================#
	# lx_1 = x + 150 # x-distance from origin to x
	# # Straight line distance from origin in top-down view to [x,y,z]
	# d_0 = np.sqrt( math.pow( (x + 150) , 2) + math.pow( (y - 150) , 2) ) 
	# theta_a = np.arcsin(lx_1 / d_0) # 1:1
	# theta_b = (np.pi / 2) - yaw # 1:2
	# theta_c = theta_a - theta_b # 1:3
	# L2 = 53.5
	# c = math.sqrt( (d_0 ** 2) +  (L2 ** 2) - ( 2 * d_0 * L2 * np.cos(theta_c) ) ) # 1:4 
	# theta_d = (np.pi / 2) - theta_a # 1:5
	# theta_e = np.arccos( ( (L2 ** 2) - (d_0 ** 2) - (c ** 2) ) / ( -2 * d_0 * c ) ) # 1:6
	# theta_f = theta_d - theta_e # 1:7
	# lx_2 = c * np.cos(theta_f) # 1:8
	# ly_2 = math.sqrt( (c ** 2) - (lx_2 ** 2) ) # 1:9
	# # 1:10
	x += 150 
	y -= 150
	z -= 10
	
	x_cen = x - (53.5 * np.cos(yaw))
	y_cen = y - (53.5 * np.sin(yaw))
	z_cen = z 


	print(x_cen)
	print(y_cen)
	print(z_cen)
	# c1 = 53.2 # last link length 

	# y_cen = y - (c1 * np.sin(yaw))
	# x_cen = x - (c1 * np.cos(yaw))
	# z_cen = z 

	# Find straight line dist from origin to cen values 
	c = math.sqrt((x_cen**2) + (y_cen**2))
	print(c)
	#===============================================================#
	# Find theta_1
	#===============================================================#
	theta_g = np.arctan(y_cen / x_cen) # 2:1
	theta_h = (np.pi / 2) - theta_g # 2:2
	theta_i = np.arcsin(110 / c) # 2:3
	theta_1 = theta_g - theta_i # 2:4
	print(np.degrees(theta_1))
	#===============================================================#
	# Find theta_6
	#===============================================================#
	# theta_j = math.acos( 110 / c ) # 3:1
	# theta_k = (math.pi / 2) - yaw # 3:2
	# theta_l = theta_j # 3:3
	# theta_m = math.pi - ( theta_k - yaw - theta_l ) # 3:4
	# theta_6 = theta_m + theta_k # 3:5
	theta_j = (np.pi / 2) - theta_1
	theta_6 = np.pi - yaw - theta_j
	#===============================================================#
	# Find [x_3end, y_3end, z_3end]
	#===============================================================#
	L4 = 437 # 4:1
	y_4 = L4 * math.sin(theta_1) # 4:2
	x_4 = L4 * math.cos(theta_1) # 4:3
	z_4 = 141 # 4:4
	# 4:5
	x_3end = x_cen + (110 * np.sin(theta_1)) + (-83 * np.cos(theta_1))
	y_3end = y_cen - (110 * np.cos(theta_1)) + (-83 * np.sin(theta_1))
	z_3end = z_4 + z_cen 
	#===============================================================#
	# Find -theta_2
	#===============================================================#
	# d_1 = math.sqrt( (x_3end ** 2) + (z_3end ** 2) ) # 5:1 
	# print(d_1)
	L1 = 152
	L3 = 244
	L5 = 213
	print(x_3end)
	print(y_3end)
	print(z_3end)
	# d_a = math.sqrt((x_3end ** 2) + (y_3end ** 2))
	# d_1 = math.sqrt( (d_a ** 2) + ((z_3end - L1) ** 2) )
	# print(d_1)

	# ret = ( (L5 ** 2) - (L3 ** 2) - (d_1 ** 2) ) / (-2 * L3 * d_1 ) 
	# print(ret)
	# eps = np.arcsin((z_3end - L1) / d_1)
	# print(eps)
	# alpha_0 = np.arccos( ( (L5 ** 2) - (L3 ** 2) - (d_1 ** 2) ) / (-2 * L3 * d_1 ) ) # 5:3
	# beta_0 = np.arccos( ( (L3 ** 2) + (L5 ** 2) - (d_1 ** 2) ) / ( 2 * L3 * L5 ) ) # 5:2
	# theta_n = np.arctan2(z_3end, x_3end) - alpha_0 # 5:4
	# theta_2 = -1 * ((np.pi / 2) - theta_n) # 5:5
	adj_len = math.sqrt((x_3end ** 2) + (y_3end ** 2))
	opp_len = z_3end - L1 
	dist = math.sqrt((adj_len ** 2) + (opp_len ** 2))
	print(dist)
	beta_1 = np.arctan2(opp_len, adj_len)
	print(beta_1)
	beta_2 = np.arccos( ((L3 ** 2) + (dist ** 2) - (L5 ** 2)) / (2 * L3 * dist) )
	print(beta_2)
	theta_2 = -1 * (beta_2 + beta_1)
	print(np.degrees(theta_2))

	# ret = ((L3 ** 2) + (L5 ** 2) - (beta_2 ** 2)) / (2 * L3 * L5)
	# print(ret)
	
	theta_x = np.arccos( ((L3 ** 2) + (L5 ** 2) - (dist ** 2)) / (2 * L3 * L5) )

	#===============================================================#
	# Find theta_3
	#===============================================================#
	theta_3 = np.pi - theta_x # 5:6
	#===============================================================#
	# Find -theta_4
	#===============================================================#
	# jeffrey = np.pi - alpha_0 - beta_0
	# jeff = (np.pi / 2) - jeffrey 
	# je = (np.pi / 2) - eps
	# theta_4 = -1 * (je - jeff) # 5:13
	theta_4 = -1 * (theta_2 + theta_3)
	#===============================================================#
	# Find theta_5
	#===============================================================#
	theta_5 = np.radians(-90)
	#===============================================================#
	theta1 = (theta_1)
	theta2 = (theta_2)
	theta3 = (theta_3)
	theta4 = (theta_4)
	theta5 = (theta_5)
	theta6 = (theta_6)
	print("\n")
	# List of thetas to test with 
	thetas = [theta1, theta2, theta3, theta4, theta5, theta6]
	# ==============================================================#
	for i in range(6): 
		print("theta " + str(i + 1) + ": " + str(np.degrees(thetas[i])))
	print("\n")
	return lab_fk(theta1, theta2, theta3, theta4, theta5, theta6)

lab_invk(0.2, 0.4, 0.05, 45)
lab_invk(0.15,-0.1, 0.25,-45)
