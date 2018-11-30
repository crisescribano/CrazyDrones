#!/usr/bin/env python

import numpy as np
from numpy import linalg 
import rospy
import std_msgs.msg
import nav_msgs.msg
import mav_msgs.msg
import geometry_msgs.msg
import rosgraph_msgs.msg
import std_srvs.srv
from std_msgs.msg import Int32, String
from geometry_msgs.msg import Twist, PoseStamped
from cf_physical_parameters import CF_parameters
import tf.transformations
import rosbag
from crazyflie_driver.msg import Position

#import yaw_controller.yaw_controller as yaw_controller

class Nav_control():

	def __init__(self):

		rospy.init_node('navigation_node_simB') 
		
		self.agent_pose= np.array([nav_msgs.msg.Odometry(),nav_msgs.msg.Odometry(),nav_msgs.msg.Odometry(),nav_msgs.msg.Odometry(),nav_msgs.msg.Odometry(),nav_msgs.msg.Odometry()])
		
		self.topic = rospy.get_param("~topic")
		self.agent_number = rospy.get_param("~agent_number")
		self.priority = rospy.get_param("~priority")

		# PUBLISHER
		#if self.agent_number != 0:
		self.force_pub = rospy.Publisher(self.topic + "/forces_input", mav_msgs.msg.TorqueThrust, queue_size = 100)

		time = rospy.get_time()

		# Trayectory to follow:
		self.trajectory = np.array([[0, 0, 0], [0, 0, 5], [4, 5, 3],[-2, 4, 2],[3, -2, 3]])
		self.timeBetweenPoints = 10
		self.sequenceCreator()
		#self.PoI = 1.5*np.array([[0, 0, 1], [1, 0, 1],[1, 1, 1],[0, 1, 1],
		#					[-1, 1, 1], [-1, 0, 1],[-1, -1, 1],[0, -1, 1], 
		#					[1, -1, 1]])

		self.counterPoI = 0
		#self.PoI = np.array([[0, 0, 1], [0, 0, 1],[0, 0, 1],[0, 0, 1]])
		#self.PoI = np.array([[0, 0, 1], [0, 0, 2],[0, 0,3],[0, 0, 4]])

		self.region_idx = 0

		#if self.agent_number == 0:
		#	self.pub_xd = rospy.Publisher('/crazyflie_0/cmd_pos', Position, queue_size = 100)

		self.con_offset = 0
		self.col_offset = 0
		self.beta_bound = 10**4#0.00000033**4
		self.coeff = np.zeros(3)
		self.a_hat = 0
		self.a_hat_dot = 0
		self.d_b_hat = 0
		self.d_b_hat_dot = 0
		self.f_b_hat = 0
		self.f_b_hat_dot = 0
		self.theta_hat = 0
		self.theta_hat_dot = 0
		self.d_con = 3
		self.r = 0.075

		self.rate = rospy.Rate(100) 

				# SUBSCRIBER
		rospy.Subscriber("/crazyflie_0/out_pos_odometry", nav_msgs.msg.Odometry, self.callback_crazyflie_0)
		rospy.Subscriber("/crazyflie_1/out_pos_odometry", nav_msgs.msg.Odometry, self.callback_crazyflie_1)
		rospy.Subscriber("/crazyflie_2/out_pos_odometry", nav_msgs.msg.Odometry, self.callback_crazyflie_2)
		rospy.Subscriber("/crazyflie_3/out_pos_odometry", nav_msgs.msg.Odometry, self.callback_crazyflie_3)
		rospy.Subscriber("/crazyflie_4/out_pos_odometry", nav_msgs.msg.Odometry, self.callback_crazyflie_4)
		rospy.Subscriber("/crazyflie_5/out_pos_odometry", nav_msgs.msg.Odometry, self.callback_crazyflie_5)

	def sequenceCreator(self):
		self.PoI = []
		for i in range(len(self.trajectory)-1):
			distance = np.linalg.norm(self.trajectory[i+1]- self.trajectory[i])
			numberPoints = self.timeBetweenPoints/0.01
			for j in range(int(numberPoints)):
				self.PoI.append(np.array([((self.trajectory[i+1, 0] - self.trajectory[i,0])/numberPoints)*j, 
									 ((self.trajectory[i+1, 1] - self.trajectory[i,1])/numberPoints)*j,
				 					 ((self.trajectory[i+1, 2] - self.trajectory[i,2])/numberPoints)*j]) + self.trajectory[i])
			#	print((self.trajectory[i+1, 0] - self.trajectory[i,0])/float(numberPoints))
			#						 ((self.trajectory[i+1, 1] - self.trajectory[i,1])/numberPoints)*j,
			#	 					 ((self.trajectory[i+1, 2] - self.trajectory[i,2])/numberPoints)*j]))

	def rot_x(self, beta):
		cos_b = np.cos(beta)
		sin_b = np.sin(beta)
		M = np.array([[1, 0, 0], [0, cos_b, -sin_b], [0, sin_b, cos_b]])
		return M

	# Rotation matrix around Y
	def rot_y(self, beta):
		cos_b = np.cos(beta)
		sin_b = np.sin(beta)
		M = np.array([[cos_b, 0, sin_b], [0, 1, 0], [-sin_b, 0, cos_b]])
		return M

	# Rotation matrix around Z
	def rot_z(self, gamma):
		cos_g = np.cos(gamma)
		sin_g = np.sin(gamma)
		M = np.array([[cos_g, -sin_g, 0], [sin_g, cos_g, 0], [0, 0, 1]])
		return M

	# Rotation matrix - from the body frame to the inertial frame
	def rot_m(self, roll, pitch, yaw):
		rotx = self.rot_x(roll)
		roty = self.rot_y(pitch)
		rotz = self.rot_z(yaw)
		rot = np.dot(np.dot(rotz, roty), rotx)	#z-y-x convention hopefully
		return rot

	

	def position_and_velocity_from_odometry(self, odometry):
		x = np.zeros(3)
		x[0] = odometry.pose.pose.position.x
		x[1] = odometry.pose.pose.position.y
		x[2] = odometry.pose.pose.position.z

		# TODO: naming of child_frame_id
		if odometry.child_frame_id == 'firefly/base_link':
			# velocity is in the body reference frame
			v_doby = np.zeros(3)
			v_body[0] = odometry.twist.twist.linear.x
			v_body[1] = odometry.twist.twist.linear.y
			v_body[2] = odometry.twist.twist.linear.z

			quaternion = np.zeros(4)
			quaternion[0] = odometry.pose.pose.orientation.x
			quaternion[1] = odometry.pose.pose.orientation.y
			quaternion[2] = odometry.pose.pose.orientation.z
			quaternion[3] = odometry.pose.pose.orientation.w
			# TODO
			rotation_matrix = tf.transformations.quaternion_from_euler(quaternion[0], quaternion[1], squaternion[2])
			#rotation_matrix = self.rot_m(quaternion[0], quaternion[1], quaternion[2])
			v = np.dot(rotation_matrix,v_body)
		else:
		# velocity is in the body reference frame
			v = np.zeros(3)
			v[0] = odometry.twist.twist.linear.x
			v[1] = odometry.twist.twist.linear.y
			v[2] = odometry.twist.twist.linear.z
		return x,v

	### Remap to change the number os the Crazyflie we refer depending on which Crazyflie runs the code:
	#		This remap is donne following the graph that is already given
	def callback_crazyflie_0(self, data):
		if self.agent_number == 0:
			self.agent_pose[0] = data
		else: 
			self.agent_pose[1] = data

	def callback_crazyflie_1(self, data): 
		if self.agent_number == 0: 
			self.agent_pose[1] = data
		elif self.agent_number == 1:
			self.agent_pose[0] = data
		else: 
			self.agent_pose[2] = data 

	def callback_crazyflie_2(self, data):  
		if self.agent_number == 0 or self.agent_number == 1: 
			self.agent_pose[2] = data
		elif self.agent_number == 2:
			self.agent_pose[0] = data
		else: 
			self.agent_pose[3] = data 

	def callback_crazyflie_3(self, data):  
		if self.agent_number == 4 or self.agent_number == 5: 
			self.agent_pose[4] = data
		elif self.agent_number == 3:
			self.agent_pose[0] = data
		else: 
			self.agent_pose[3] = data 

	def callback_crazyflie_4(self, data):  
		if self.agent_number == 5: 
			self.agent_pose[5] = data
		elif self.agent_number == 4:
			self.agent_pose[0] = data
		else: 
			self.agent_pose[4] = data 

	def callback_crazyflie_5(self, data): 
		if self.agent_number == 5:
			self.agent_pose[0] = data
		else: 
			self.agent_pose[5] = data 

	def eta_funtion(self, x, x_other, v, v_other):
		preop_x = np.array([x[0]-x_other[0], x[1]-x_other[1], x[2]-x_other[2]])
		preop_v = np.array([v[0]-v_other[0], v[1]-v_other[1], v[2]-v_other[2]])
		eta_con = self.d_con**2 - np.linalg.norm(preop_x)**2     
		eta_con_dot = - 2*np.dot(preop_x, preop_v)
		return eta_con, eta_con_dot

	def iota_con(self, x, x_other, v, v_other):
		preop_x = np.array([x[0]-x_other[0], x[1]-x_other[1], x[2]-x_other[2]])
		preop_v = np.array([v[0]-v_other[0], v[1]-v_other[1], v[2]-v_other[2]])
		iota_con = np.linalg.norm(preop_x)**2 - 4*self.r**2
		iota_col_dot = 2*np.dot(preop_x, preop_v)
		return iota_con, iota_col_dot

	def A_con(self):
		A = np.array([[self.con_offset**5,self.con_offset**4, self.con_offset**3],[5*self.con_offset**4, 4*self.con_offset**3, 3*self.con_offset**2], [20*self.con_offset**3, 12*self.con_offset**2, 6*self.con_offset]])
		return A

	def B_con(self):
		B = np.array([self.beta_bound, 0, 0])
		return B

	def A_col(self):
		A = np.array([[self.col_offset**5,self.col_offset**4, self.col_offset**3],[5*self.col_offset**4, 4*self.col_offset**3, 3*self.col_offset**2], [20*self.col_offset**3, 12*self.col_offset**2, 6*self.col_offset]])
		return A

	def B_col(self):
		B = np.array([self.beta_bound, 0, 0])
		return B

	def beta_con(self, eta_con, eta_con_dot):
		beta_con = self.coeff[0]*eta_con**5 + self.coeff[1]*eta_con**4 + self.coeff[2]*eta_con**3
		beta_con_dot = (5*self.coeff[0]*eta_con**4 + 4*self.coeff[1]*eta_con**3 + 3*self.coeff[2]*eta_con**2) * eta_con_dot
		return beta_con, beta_con_dot

	def grad_beta_con(self, beta_con, beta_con_dot, eta_con, eta_con_dot, x, x_other, v, v_other):
		preop_x = np.array([x[0]-x_other[0], x[1]-x_other[1], x[2]-x_other[2]])
		preop_v = np.array([v[0]-v_other[0], v[1]-v_other[1], v[2]-v_other[2]])
		grad_beta_con = -1/(beta_con + 1e-12)**2*(5*self.coeff[0]*eta_con**4 + 4*self.coeff[1]*eta_con**3 + 3*self.coeff[2]*eta_con**2)*(-2*(preop_x))
		term1 = 2/(beta_con + 1e-12)**3 * beta_con_dot * (5*self.coeff[0]*eta_con**4 + 4*self.coeff[1]*eta_con**3 + 3*self.coeff[2]*eta_con**2)*(-2*(preop_x))
		term2 = -1/(beta_con + 1e-12)**2 * (20*self.coeff[0]*eta_con**3 + 12*self.coeff[1]*eta_con**2 + 6*self.coeff[2]*eta_con)*eta_con_dot*(-2*(preop_x))
		term3 = -1/(beta_con + 1e-12)**2*(5*self.coeff[0]*eta_con**4 + 4*self.coeff[1]*eta_con**3 + 3*self.coeff[2]*eta_con**2)*(-2*(preop_v))
		grad_beta_con_dot = term1 + term2 + term3
		return grad_beta_con, grad_beta_con_dot
			

	def reset_grad_beta(self):
		grad_beta = np.zeros(3)
		grad_beta_dot = np.zeros(3)
		return grad_beta, grad_beta_dot

	def beta_col(self, iota_col, iota_col_dot):
		beta_col = self.coeff[0]*iota_col**5 + self.coeff[1]*iota_col**4 + self.coeff[2]*iota_col**3
		beta_col_dot = (5*self.coeff[0]*iota_col**4 + 4*self.coeff[1]*iota_col**3 + 3*self.coeff[2]*iota_col**2)*iota_col_dot
		return beta_col, beta_col_dot

	def grad_beta_col(self, beta_col, beta_col_dot, iota_col, iota_col_dot, x, x_other, v, v_other):
		preop_x = preop_x = np.array([x[0]-x_other[0], x[1]-x_other[1], x[2]-x_other[2]])
		preop_v = np.array([v[0]-v_other[0], v[1]-v_other[1], v[2]-v_other[2]])
		grad_beta_col = -1/beta_col**2*(5*self.coeff[0]*iota_col**4 + 4*self.coeff[1]*iota_col**3 + 3*self.coeff[2]*iota_col**2)*(2*(preop_x))
		term1 = 2/beta_col**3 * beta_col_dot * (5*self.coeff[0]*iota_col**4 + 4*self.coeff[1]*iota_col**3 + 3*self.coeff[2]*iota_col**2)*(2*(preop_x))
		term2 = -1/beta_col**2 * (20*self.coeff[0]*iota_col**3 + 12*self.coeff[1]*iota_col**2 + 6*self.coeff[2]*iota_col)*iota_col_dot*(2*(preop_x))
		term3 = -1/beta_col**2*(5*self.coeff[0]*iota_col**4 + 4*self.coeff[1]*iota_col**3 + 3*self.coeff[2]*iota_col**2)*(2*(preop_v))
		grad_beta_col_dot = term1 + term2 + term3 
		return grad_beta_col, grad_beta_col_dot


	def navigation(self):

		#GAINS
		#kp_x = .05 #0.5 for 10     ..*0.05 
		
		kp_x = 2# 2
		ki_x = 0*0.1

		kp_vx = 0.005#25
		ki_vx = 0.00001#1 

		kp_y = kp_x#2
		ki_y = ki_x

		kp_vy = kp_vx#25
		ki_vy = ki_vx#1  

		kp_z = 2#2#2
		ki_z = 0.5#0.5#2
		
		kp_vz = 0.1
		ki_vz = 0.002#15#0.5

		k_e_tilde = 0.001#5
		lambda_int = 0.0000015#0.00015
		k_theta = .001#0.01
		k_f_b = 0
		k_d_b = 0

		k_connect = 0 # Beta for connetivity

		if self.agent_number == 0:
			ki = 1
			k_y_tet= 1
			k_dis = 1
		else:
			ki = 5 # Velocidades
			k_dis = 1 # Termino diss
			k_y_tet = 1 # Termino Y*theta

		mass = 0.027
		dt = 0.01
		
		integrator_pos = np.zeros(3)
		integrator_v = np.zeros(3)
		navigation_term = np.zeros(3)
		dissip_term = np.zeros(3)
		counter = 1
		mode = 0
		second_reg = False
		grav = 9.81

		# Initiation etas and iotas 
		eta_con = np.zeros(3)   
		eta_con_dot = np.zeros(3) 
		beta_con = np.zeros(3) 
		beta_con_dot = np.zeros(3)
		grad_beta_con = [np.zeros(3),np.zeros(3),np.zeros(3)]
		grad_beta_con_dot = [np.zeros(3),np.zeros(3),np.zeros(3)]

		iota_col = np.zeros(5)
		iota_col_dot = np.zeros(5)
		beta_col = np.zeros(5) 
		beta_col_dot = np.zeros(5)
		grad_beta_col = [np.zeros(3),np.zeros(3),np.zeros(3),np.zeros(3),np.zeros(3),np.zeros(3)]
		grad_beta_col_dot = [np.zeros(3),np.zeros(3),np.zeros(3),np.zeros(3),np.zeros(3),np.zeros(3)] 

		beta_term_con = np.zeros(3)
		beta_term_col = np.zeros(5) 

		beta_term_con_dot = np.zeros(3)
		beta_term_col_dot = np.zeros(6)

		navigation_term = np.zeros(3)
		dissip_term = np.zeros(3)
		e_tilde = np.zeros(3)
		
		# Control:
		control = np.zeros(3)
		# Duda:
		e3 = np.array([0.0,0.0,1.0])
		# Error
		ep = np.zeros(3)

		x = [np.zeros(3),np.zeros(3),np.zeros(3),np.zeros(3),np.zeros(3),np.zeros(3)]
		v = [np.zeros(3),np.zeros(3),np.zeros(3),np.zeros(3),np.zeros(3),np.zeros(3)]

		while not rospy.is_shutdown():
			
			# Get the position and velocity of all the CrazyFlies
			for i in range(0, 6):
				x[i],v[i] = self.position_and_velocity_from_odometry(self.agent_pose[i])


			# Check if we are lider o follower
			if self.priority == 1: # Lider
				mode = 1 
			else:                  # Follower
				mode = 0

			# Integration
			self.a_hat = self.a_hat + dt*self.a_hat_dot
			self.d_b_hat = self.d_b_hat + dt*self.d_b_hat_dot
			self.f_b_hat = self.f_b_hat + dt*self.f_b_hat_dot
			self.theta_hat = self.theta_hat + dt*self.theta_hat_dot

			# Point to achieve
			if(self.region_idx >= self.PoI):
				self.region_idx = 0
			xd = self.PoI[self.region_idx]
			self.region_idx += 1
			###########################################
			### Achieve desired point by the leader ###
			###########################################

			if mode == 1:

				ep = x[0] - xd
				integrator_pos = integrator_pos + dt*ep
				# if np.linalg.norm(ep) < 0.05:	# Cheqck if desired point is achieved
				# 	if (self.region_idx < len(self.PoI)-1) and self.counterPoI < 100:
				# 		self.counterPoI += 1
				# 		print ('self.counterPoI ' + str(self.counterPoI ))
				# 	elif (self.region_idx < len(self.PoI)-1) and self.counterPoI >= 100:
				# 		print ('REACHED!!! POINT NUMBER ' + str(self.region_idx))
				# 		self.region_idx+= 1
				# 		self.counterPoI = 0
				# 		if self.region_idx == len(self.PoI)-1: 
				# 		#print ('REACHED!!! POINT NUMBER ' + str(self.region_idx))
				# 			self.region_idx = 0
				# 		# Another point to achieve
				# 	else:
				# 		self.counterPoI = 0
					#integrator_pos = np.zeros(3)	# Reset the integrator

			###############################
			### Value of etas and iotas ###
			###############################

			# Define the conections in the graph
			# Edges = {(1,2), (1,3), (1,4), (3,4), (3,5), (5,6), (6,2)}
			# Edges = {(0,1), (0,2), (0,3), (2,3), (2,4), (4,5), (5,1)}

			# Definition of etas depending on the Crazyflie we are
			if self.agent_number == 0:
				eta_con[0], eta_con_dot[0] = self.eta_funtion(x[0], x[1], v[0], v[1])
				eta_con[1], eta_con_dot[1] = self.eta_funtion(x[0], x[2], v[0], v[2])
				eta_con[2], eta_con_dot[2] = self.eta_funtion(x[0], x[3], v[0], v[3])

			if self.agent_number == 1:
				eta_con[0], eta_con_dot[0] = self.eta_funtion(x[0], x[1], v[0], v[1])
				eta_con[1], eta_con_dot[1] = self.eta_funtion(x[0], x[5], v[0], v[5])

			if self.agent_number == 2:
				eta_con[0], eta_con_dot[0] = self.eta_funtion(x[0], x[1], v[0], v[1])
				eta_con[1], eta_con_dot[1] = self.eta_funtion(x[0], x[3], v[0], v[3])
				eta_con[2], eta_con_dot[2] = self.eta_funtion(x[0], x[4], v[0], v[4])

			if self.agent_number == 3:
				eta_con[0], eta_con_dot[0] = self.eta_funtion(x[0], x[1], v[0], v[1])
				eta_con[1], eta_con_dot[1] = self.eta_funtion(x[0], x[3], v[0], v[3])

			if self.agent_number == 4:
				eta_con[0], eta_con_dot[0] = self.eta_funtion(x[0], x[3], v[0], v[3])
				eta_con[1], eta_con_dot[1] = self.eta_funtion(x[0], x[5], v[0], v[5])

			if self.agent_number == 5:
				eta_con[0], eta_con_dot[0] = self.eta_funtion(x[0], x[5], v[0], v[5])
				eta_con[1], eta_con_dot[1] = self.eta_funtion(x[0], x[2], v[0], v[2])

			# Definition of the posible collision (everyone with everyone)

			for i in range(0, 5):
				iota_col[i],iota_col_dot[i] = self.iota_con(x[0], x[i+1], v[0], v[i+1])
			
			####################################################################################
			### Implementation of not disconnection: ETA BIGGER THAN 0 FOR NOT DISCONNECTION ###
			####################################################################################

			con_distance_meas = 0 								# Inter agent distance where we start taking conection into account (greater than)
			self.con_offset = self.d_con**2 - con_distance_meas**2	

			for i in range(0, 3):

				if i!=2 or self.agent_number == 0 or self.agent_number == 2:

					if False:#eta_con[i] < 0:		# Real distance btw CF is more than d_con they can not see each other
						print(self.topic + "NOT CONNECTED WITH " + str(i))
						beta_con[i] = 0
						#beta_con_dot[i] = 0
						grad_beta_con[i],grad_beta_con_dot[i] = self.reset_grad_beta()

					elif eta_con[i] < self.con_offset + 10000000000000:	# If the distante between Crazyflie is more that 0, define betas

						A = self.A_con()
						B = self.B_con()
						self.coeff = np.dot(np.linalg.inv(A),B)

						beta_con[i],beta_con_dot[i] = self.beta_con(eta_con[i], eta_con_dot[i])
						grad_beta_con[i],grad_beta_con_dot[i] = self.grad_beta_con(beta_con[i], beta_con_dot[i], eta_con[i], eta_con_dot[i], x[i], x[i+1], v[i], v[i+1])
						
					else:								# If the distance between Crazyflies is less than 0:
						beta_con[i] = self.beta_bound
						grad_beta_con[i],grad_beta_con_dot[i] = self.reset_grad_beta()
						#print("DISTANCIA ENTRE CF MENOR QUE CERO000000000000000000000000000000000000000")
		
			#############################################################################
			### Implementation of not collision: IOTA BIGGER THAT 0 FOR NOT COLLISION ###
			#############################################################################

			# Iota = real distance between Crazuflies: | * |----------| * |     (just -----)
			col_distance_meas = self.d_con 							# Inter agent distance where we start taking collision into account (less than)
			self.col_offset = col_distance_meas**2 - 4*self.r**2
			
			for i in range(0, 5):

				if False:#iota_col[i] <= 0:						# If iota less than 0, everything 0
					beta_col[i] = 0
					beta_col_dot[i] = 0
					grad_beta_col[i],grad_beta_col_dot[i] = self.reset_grad_beta()
				        
				elif iota_col[i] <= self.col_offset + 10000000000:		# If iota is smaller than a value, define betas

					A = self.A_col()
					B = self.B_col()
					self.coeff = np.dot(np.linalg.inv(A),B)

					beta_col[i],beta_col_dot[i] = self.beta_col(iota_col[i], iota_col_dot[i])
					grad_beta_col[i],grad_beta_col_dot[i] = self.grad_beta_col(beta_col[i] , beta_col_dot[i], iota_col[i], iota_col_dot[i], x[i], x[i+1], v[i], v[i+1])
					
				else:									# If iota is bigger than a value:
					beta_col[i] = self.beta_bound
					grad_beta_col[i],grad_beta_col_dot[i] = self.reset_grad_beta()

			########################
			### Desired velocity ###
			########################
			# Vi = Ki * SUM alfa * grad_beta, as alfa is (-1, 1, 0) if i=m1, i=m2, i!=m1,m2  
			#	respectively in the specific Crazyflie alfa = -1
			beta_term_con = -grad_beta_con[0] - grad_beta_con[1] #- grad_beta_con[2] 
			beta_term_col = np.zeros(3)#-grad_beta_col[0] - grad_beta_col[1]#- grad_beta_col[2] #- grad_beta_col[3]# - grad_beta_col[4] - grad_beta_col[5]

			beta_term_con_dot =  -grad_beta_con_dot[0] - grad_beta_con_dot[1] #- grad_beta_con_dot[2] 
			beta_term_col_dot = np.zeros(3)#-grad_beta_col_dot[0] - grad_beta_col_dot[1] #- grad_beta_col_dot[2] #- grad_beta_col_dot[3] #- grad_beta_col_dot[4] - grad_beta_col_dot[5]

			if mode == 1:	# If the Crazyflie is the leader
				v_des = np.array([-kp_x*ep[0] - ki_x*integrator_pos[0], -kp_y*ep[1] - ki_y*integrator_pos[1], -kp_z*ep[2] - ki_z*integrator_pos[2]])  #+ ki*(beta_term_col + beta_term_con) 

			 	v_des_dot = np.array([-kp_vx*v[0][0], -kp_vy*v[0][1], -kp_vz*v[0][2]])# + ki*(beta_term_col_dot + beta_term_con_dot) #- lambda_int*ep

			else:
				v_des = ki*(beta_term_col + beta_term_con) 
				# Calcutale the velocity error:
				v_des_dot = ki*(beta_term_col_dot + beta_term_con_dot) 
			
			#v_des = np.array([10, 0, v_des[2]])
			e_v = v[0] - v_des
			integrator_v = integrator_v + e_v*dt
			# Dissipative terms:
			dissip_term = np.array([kp_vx*e_v[0] + ki_vx*integrator_v[0], kp_vy*e_v[1] + ki_vy*integrator_v[1], kp_vz*e_v[2] + ki_vz*integrator_v[2]])
			# rospy.loginfo("e_v " + str(e_v))
			# rospy.loginfo("ep " + str(ep))
			# Calculate estimations needed:
			if mode == 1:
				e_tilde = ep + lambda_int*integrator_pos
			else:
				e_tilde = np.zeros(3)

			# Calculate Y matrix:#Y = np.array( [v_des_dot[0],v_des_dot[1],v_des_dot[2]+grav])
			Y = v_des_dot + [0, 0, grav]

			#print("v_des: ", v_des)

			#print(1/2*np.linalg.norm(ep)**2)
			#print(1/2*mass*np.linalg.norm(e_v)**2 )
			#print('energy = ', .5**np.linalg.norm(ep)**2 + .5*mass*np.linalg.norm(e_v)**2 )
			# Calculate term of control:
			#gravity compensation: k_y_tet*Y*self.theta_hat np.array([0,0,mass*grav])

			#print('F_inertial = ', control)
			control = 0*beta_term_col + 0*beta_term_con - dissip_term + 0*Y*self.theta_hat - k_e_tilde*e_tilde  #- np.sign(e_v)*np.linalg.norm(v[0],1)*self.f_b_hat - np.sign(e_v)*self.d_b_hat
			#control = beta_term_col + b=eta_term_con - dissip_term + Y*self.theta_hat - k_e_tilde*e_tilde #- np.sign(e_v)*np.linalg.norm(v[self.agent_number],1)*self.f_b_hat
			

			#control = - k_dis*dissip_term + Y*self.theta_hat - k_e_tilde*e_tilde + beta_term_col + k_connect*beta_term_con	
			#rospy.loginfo("Control : " + str(control)) 
			#rospy.loginfo("error v : " + str(e_v[0])) 
			#rospy.loginfo("error p : " + str(ep[0])) 
			#print("control: ", control)
			#print("x[0]: ", x[0])

			#control = beta_term_col + beta_term_con - dissip_term + Y*self.theta_hat - k_e_tilde*e_tilde
			#control = (control/0.027)*dt**2
			#print(control)
			# if self.agent_number == 1:
			# #	print("position follower = ", x[0])
			# 	print("beta_term_con de 1 = ", k*beta_term_con)
			# 	print("dissip_term de 1 = ", -k_dis*dissip_term)
			# 	print("Y*self.theta_hat de 1 = ", k_y_tet*Y*self.theta_hat)
			# 	print("######################################################################")
			# if self.agent_number == 0:
			# 	print("control: " + str(control))
			# 	print("position follower = ", x[0])
			# 	print("beta_term_con de 0 = ", k_connect*beta_term_con)
			# 	print("dissip_term de 0 = ", -k_dis*dissip_term)
			# 	print("Y*self.theta_hat de 0 = ", k_y_tet*Y*self.theta_hat)
			# 	print("Y de 0 = ", k_y_tet*Y)
			# 	print("-k_theta*np.dot(Y,e_v) de 0 = ", -k_theta*np.dot(Y,e_v))
			# 	print("- k_e_tilde*e_tilde de 0 = ", - k_e_tilde*e_tilde)
			# 	print("######################################################################")

			# 	print("e_v de 1 = ", e_v)
			# 	print("control value for 1 is: " + str(control))
			# 	print("eta0 de 1 is: " , eta_con[0])
			

			# Calculate discrepance terms:
			self.d_b_hat_dot = k_d_b*np.linalg.norm(e_v)
			self.f_b_hat_dot = k_f_b*np.linalg.norm(e_v,1)*np.linalg.norm(v[0])
			self.theta_hat_dot = -k_theta*np.dot(Y,e_v)

			# Publish several messages:
			# 	# Publish lider errors (GUESS)
			# if mode == 1:
			# 	e_msg = std_msgs.msg.Float64()
			# 	e_msg.data = np.linalg.norm( np.concatenate((e_tilde,e_v)) )
			# 	self.e_p_pub.publish(e_msg)

				# Publish theta, d and f
			#theta_hat_msg = std_msgs.msg.Float64()
			#theta_hat_msg.data = np.linalg.norm(self.theta_hat)
			#self.theta_hat_pub.publish(theta_hat_msg)

			#d_hat_msg = std_msgs.msg.Float64()
			#d_hat_msg.data = self.d_b_hat
			#self.d_hat_pub.publish(d_hat_msg)

			#f_hat_msg = std_msgs.msg.Float64()
			#f_hat_msg.data = self.f_b_hat
			#self.f_hat_pub.publish(f_hat_msg)
			#if self.agent_number != 0:
				# Publish control
			mesage_to_pub = mav_msgs.msg.TorqueThrust()
			mesage_to_pub.thrust.x = control[0] 
			mesage_to_pub.thrust.y = control[1]
			mesage_to_pub.thrust.z = control[2]
			self.force_pub.publish(mesage_to_pub)

				# Publish 
			# tmp_mesage_to_pub = mav_msgs.msg.TorqueThrust()
			# tmp_mesage_to_pub.thrust.x = x_other_1[0]
			# tmp_mesage_to_pub.thrust.y = x_other_1[1]
			# tmp_mesage_to_pub.thrust.z = x_other_1[2]
   #  		self.temp_pub.publish(tmp_mesage_to_pub)

			self.rate.sleep()

if __name__ == '__main__':
    control = Nav_control()
    
    control.navigation()
    rospy.spin()