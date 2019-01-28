#!/usr/bin/env python

import re
import os
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


dataStore = True
#import yaw_controller.yaw_controller as yaw_controller

class Nav_control():

	def __init__(self):

		rospy.init_node('navigation_node_simB')
			
		self.topic = rospy.get_param("~topic")
		self.numberQuads = rospy.get_param("~number_quads")
		self.agent_number = rospy.get_param("~agent_number")
		connections = rospy.get_param("~connections")
		self.priority = rospy.get_param("~priority")

		self.graphCon = list(map(int, re.findall(r"[\w']+", connections)))
		dumbList = list(self.graphCon)
		for index in dumbList:
			if index >= (self.numberQuads):
				self.graphCon.remove(index)
		print "Crazyflie " + str(self.agent_number) + " is connected to : " + str(self.graphCon)

		self.force_pub = rospy.Publisher(self.topic + "/forces_input", mav_msgs.msg.TorqueThrust, queue_size = 100)
		if dataStore:
			self.beta_pub = rospy.Publisher(self.topic + "/betas_input", PoseStamped, queue_size = 100)
			if self.priority:
				self.traj_pub = rospy.Publisher("/trajectory", PoseStamped, queue_size = 100)

		time = rospy.get_time()

		# Trayectory to follow:
		self.trajectory = np.array([[0, 0, 0.2],[0, 0, 2], [0, 0, 5], [4, 5, 3],[-2, 4, 2],[3, -2, 3],[1, -1, 2], [0, 0, 0]])
		
		#self.PoI = 1.5*np.array([[0, 0, 1], [1, 0, 1],[1, 1, 1],[0, 1, 1],
		#					[-1, 1, 1], [-1, 0, 1],[-1, -1, 1],[0, -1, 1], 
		#					[1, -1, 1]])
		#self.PoI = np.array([[0, 0, 1], [0, 0, 1],[0, 0, 1],[0, 0, 1]])
		#self.PoI = np.array([[0, 0, 1], [0, 0, 2],[0, 0,3],[0, 0, 4]])

		self.timeBetweenPoints = 15
		self.counterPoI = 0
		self.region_idx = 0
		self.con_offset = 0
		self.col_offset = 0
		#self.beta_bound_col = 1000000000000000#0
		#self.beta_bound_col = 1000000000000 # 0
		self.beta_bound_con = 500000# 0
		self.beta_bound_col = 5000000000
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
		self.r = 0.2

		self.time_store = None
		self.pos_store = None
		self.beta_con_store = None
		self.beta_col_store = None
		self.control_store = None
		self.dissip_term_store = None
		self.ep_store = None
		self.e_tilde_store = None
		self.e_v_store = None
		self.v_des_store = None
		self.v_des_dot_store = None

		self.rate = rospy.Rate(100) 
		self.agent_pose = []
		for i in range(self.numberQuads):
			try:
				rospy.loginfo("Creating callback: callback_crazyflie_" + str(i))
				rospy.Subscriber("/crazyflie_" + str(i) + "/out_pos_odometry", nav_msgs.msg.Odometry, self.callback_crazyflie)
				self.agent_pose.append(nav_msgs.msg.Odometry())
									
			except AttributeError:
				print "callback_crazyflie_0 not found"

		self.sequenceCreator()
		#rospy.on_shutdown(self.save_data)

	def sequenceCreator(self):
		self.PoI = []
		for i in range(len(self.trajectory)-1):
			distance = np.linalg.norm(self.trajectory[i+1]- self.trajectory[i])
			numberPoints = self.timeBetweenPoints/0.01
			for j in range(int(numberPoints)):
				self.PoI.append(np.array([((self.trajectory[i+1, 0] - self.trajectory[i,0])/numberPoints)*j, 
									 ((self.trajectory[i+1, 1] - self.trajectory[i,1])/numberPoints)*j,
									 ((self.trajectory[i+1, 2] - self.trajectory[i,2])/numberPoints)*j]) + self.trajectory[i])

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

		v = np.zeros(3)
		v[0] = odometry.twist.twist.linear.x
		v[1] = odometry.twist.twist.linear.y
		v[2] = odometry.twist.twist.linear.z

		return x,v

	def callback_crazyflie(self, data):
		self.agent_pose[int(data.header.frame_id)] = data

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
		B = np.array([self.beta_bound_con, 0, 0])
		return B

	def A_col(self):
		A = np.array([[self.col_offset**5,self.col_offset**4, self.col_offset**3],[5*self.col_offset**4, 4*self.col_offset**3, 3*self.col_offset**2], [20*self.col_offset**3, 12*self.col_offset**2, 6*self.col_offset]])
		return A

	def B_col(self):
		B = np.array([self.beta_bound_col, 0, 0])
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

		kp_x = 1# 2
		ki_x = 0#0.001

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
		k_theta = .0001#0.01
		k_f_b = 0
		k_d_b = 0

		k_connect = 0 # Beta for connetivity
		k_hor = 0.9
		if self.priority == 1:
			ki_con = 300000#50
			ki_col = 100000#50
			k_y_tet= 1
			k_dis = 1
		else:
			ki_con = 300000#100000#*1000# Velocidades
			ki_col = 100000# Velocidades
			ki_con = ki_con
			k_dis = 1 # Termino diss
			k_y_tet = 1 # Termino Y*theta

		mass = 0.027
		dt = 0.01
		
		integrator_pos = np.zeros(3)
		integrator_v = np.zeros(3)
		navigation_term = np.zeros(3)
		dissip_term = np.zeros(3)
		counter = 1
		second_reg = False
		grav = 9.81

		# Initiation etas and iotas 
		eta_con = {}   
		eta_con_dot = {} 
		beta_con = {} 
		beta_con_dot = {}
		grad_beta_con = {}
		grad_beta_con_dot = {}
		for i in range(len(self.graphCon)):
			eta_con[self.graphCon[i]] = 0  
			eta_con_dot[self.graphCon[i]] = 0
			beta_con[self.graphCon[i]] = 0
			beta_con_dot[self.graphCon[i]] = 0
			grad_beta_con[self.graphCon[i]] = np.zeros(3)
			grad_beta_con_dot[self.graphCon[i]] =np.zeros(3)

		iota_col = {}
		iota_col_dot = {}
		beta_col = {}
		beta_col_dot = {}
		grad_beta_col = {}
		grad_beta_col_dot = {}
		x = []
		v = []

		for i in range(self.numberQuads):
			x.append(np.zeros(3))
			v.append(np.zeros(3))
			if not i == self.agent_number:
				iota_col[i] = 0
				iota_col_dot[i] = 0
				beta_col[i] = 0
				beta_col_dot[i] = 0
				grad_beta_col[i] = np.zeros(3)
				grad_beta_col_dot[i] = np.zeros(3)
			
		beta_term_con = np.zeros(3)
		beta_term_col = np.zeros(3) 

		beta_term_con_dot = np.zeros(3)
		beta_term_col_dot = np.zeros(3)

		navigation_term = np.zeros(3)
		dissip_term = np.zeros(3)
		e_tilde = np.zeros(3)
		
		# Control:
		control = np.zeros(3)
		
		# Error
		ep = np.zeros(3)

		if dataStore:
			counterTraj = 0

		while not rospy.is_shutdown():
			# Get the position and velocity of all the CrazyFlies
			for i in range(self.numberQuads):
				x[i],v[i] = self.position_and_velocity_from_odometry(self.agent_pose[i])
			
			# Integration
			self.a_hat = self.a_hat + dt*self.a_hat_dot
			self.d_b_hat = self.d_b_hat + dt*self.d_b_hat_dot
			self.f_b_hat = self.f_b_hat + dt*self.f_b_hat_dot
			self.theta_hat = self.theta_hat + dt*self.theta_hat_dot

			# Point to achieve
			if(self.region_idx >= len(self.PoI)-1):
				self.region_idx = 0
			xd = self.PoI[self.region_idx]
			self.region_idx += 1


			###########################################
			### Achieve desired point by the leader ###
			###########################################

			if self.priority == 1:
				if dataStore:
					if counterTraj < 10:
						counterTraj = counterTraj + 1
					if counterTraj == 10:
						msg_traj = PoseStamped()
						msg_traj.pose.position.x = xd[0]
						msg_traj.pose.position.y = xd[1]
						msg_traj.pose.position.z = xd[2]
						msg_traj.header.stamp = rospy.Time.now()
						self.traj_pub.publish(msg_traj)

				ep = x[self.agent_number] - xd
				integrator_pos = integrator_pos + dt*ep
				

			###############################
			### Value of etas and iotas ###
			###############################

			# Define the conections in the graph
			# Edges = {(0,1), (0,2), (0,3), (2,3), (2,4), (4,5), (5,1)}
			for index in self.graphCon:
				eta_con[index], eta_con_dot[index] = self.eta_funtion(x[self.agent_number], x[index], v[self.agent_number], v[index])

			# Definition of the posible collision (everyone with everyone)
			for i in range(self.numberQuads):
				if not i == self.agent_number:
					iota_col[i],iota_col_dot[i] = self.iota_con(x[self.agent_number], x[i], v[self.agent_number], v[i])
			
			####################################################################################
			### Implementation of not disconnection: ETA BIGGER THAN 0 FOR NOT DISCONNECTION ###
			####################################################################################

			con_distance_meas = 0 								# Inter agent distance where we start taking conection into account (greater than)
			self.con_offset = self.d_con**2 - con_distance_meas**2	

			for index in self.graphCon:
				if eta_con[index] < 0:		# Real distance btw CF is more than d_con they can not see each other
					#print(self.topic + " NOT CONNECTED WITH " + str(index))
					beta_con[index] = 0
					#beta_con_dot[i] = 0
					grad_beta_con[index],grad_beta_con_dot[index] = self.reset_grad_beta()

				elif eta_con[index] < self.con_offset:	# If the distante between Crazyflie is more that 0, define betas
					#print(self.topic + " CONNECTED WITH " + str(index))
					A = self.A_con()
					B = self.B_con()
					self.coeff = np.dot(np.linalg.inv(A),B)

					beta_con[index],beta_con_dot[index] = self.beta_con(eta_con[index], eta_con_dot[index])
					grad_beta_con[index],grad_beta_con_dot[index] = self.grad_beta_con(beta_con[index], 
																					   beta_con_dot[index], 
																					   eta_con[index], 
																					   eta_con_dot[index], 
																					   x[self.agent_number], 
																					   x[index], 
																					   v[self.agent_number], 
																					   v[index])
					
				else:								# If the distance between Crazyflies is less than 0:
					beta_con[index] = self.beta_bound_con
					grad_beta_con[index],grad_beta_con_dot[index] = self.reset_grad_beta()

		
			#############################################################################
			### Implementation of not collision: IOTA BIGGER THAT 0 FOR NOT COLLISION ###
			#############################################################################

			# Iota = real distance between Crazuflies: | * |----------| * |     (just -----)
			col_distance_meas = self.d_con 							# Inter agent distance where we start taking collision into account (less than)
			self.col_offset = col_distance_meas**2 - 4*self.r**2
			
			for i in range(self.numberQuads):
				if not i == self.agent_number:
					if iota_col[i] <= 0:
						rospy.loginfo(self.topic + " COLLIDED WITH " + str(i) + " with distance: " + str(np.linalg.norm(x[i] - x[self.agent_number])))						# If iota less than 0, everything 0
						beta_col[i] = 0
						beta_col_dot[i] = 0
						grad_beta_col[i],grad_beta_col_dot[i] = self.reset_grad_beta()
						
					elif iota_col[i] <= self.col_offset:		# If iota is smaller than a value, define betas

						A = self.A_col()
						B = self.B_col()
						self.coeff = np.dot(np.linalg.inv(A),B)

						beta_col[i],beta_col_dot[i] = self.beta_col(iota_col[i], iota_col_dot[i])
						grad_beta_col[i],grad_beta_col_dot[i] = self.grad_beta_col(beta_col[i],
																				   beta_col_dot[i], 
																				   iota_col[i], 
																				   iota_col_dot[i], 
																				   x[self.agent_number], 
																				   x[i], 
																				   v[self.agent_number], 
																				   v[i])
						
					else:
						#rospy.loginfo(self.topic + " IOTA BIG: COLLIDED WITH " + str(i))
						beta_col[i] = self.beta_bound_col
						grad_beta_col[i],grad_beta_col_dot[i] = self.reset_grad_beta()

			########################
			### Desired velocity ###
			########################

			beta_term_con_dot = np.zeros(3)
			beta_term_con = np.zeros(3)

			for index in self.graphCon:
				#print("Adding betas of " + str(self.agent_number) + " of crazyflie " + str(index))
				beta_term_con -= grad_beta_con[index]
				beta_term_con_dot -= grad_beta_con_dot[index]

			beta_term_col_dot = np.zeros(3)
			beta_term_col = np.zeros(3)

			for i in range(self.numberQuads):
				if not i == self.agent_number:
					#print("Adding col betas of " + str(self.agent_number) + " of crazyflie " + str(i))
					beta_term_col -= grad_beta_col[i]
					beta_term_col_dot -= grad_beta_col_dot[i]

			if(x[self.agent_number][2] <= 0.2):
				k_col_height = 0.35
				k_con_height = 0.35
			else:
				k_col_height = 1.0
				k_con_height = 1.0

			if self.priority == 1:	# If the Crazyflie is the leader
				v_des = np.array([-kp_x*ep[0] - ki_x*integrator_pos[0], -kp_y*ep[1] - ki_y*integrator_pos[1], -kp_z*ep[2] - ki_z*integrator_pos[2]]) + (k_col_height*ki_col*beta_term_col + k_con_height*ki_con*beta_term_con)
				v_des_dot = np.array([-kp_vx*v[self.agent_number][0], -kp_vy*v[self.agent_number][1], -kp_vz*v[self.agent_number][2]]) + (k_col_height*ki_col*beta_term_col_dot + k_con_height*ki_con*beta_term_con_dot)  #- lambda_int*ep

				e_v = v[self.agent_number] - v_des
				integrator_v = integrator_v + e_v*dt
				
				dissip_term = np.array([k_hor*(kp_vx*e_v[0] + ki_vx*integrator_v[0]), k_hor*(kp_vy*e_v[1] + ki_vy*integrator_v[1]), kp_vz*e_v[2] + ki_vz*integrator_v[2]])
				
				e_tilde = ep + lambda_int*integrator_pos

				Y = v_des_dot + [0, 0, grav]

				control = k_col_height*beta_term_col + k_con_height*beta_term_con - dissip_term + 0*Y*self.theta_hat - k_e_tilde*e_tilde  #- np.sign(e_v)*np.linalg.norm(v[0],1)*self.f_b_hat - np.sign(e_v)*self.d_b_hat
				#control = beta_term_col + b=eta_term_con - dissip_term + Y*self.theta_hat - k_e_tilde*e_tilde #- np.sign(e_v)*np.linalg.norm(v[self.agent_number],1)*self.f_b_hat
				#rospy.loginfo("Control of " + self.topic + ": " + str(control) 
										   #+ "\n v_des : " + str(v_des)
										   #+ "\n beta_term_con : " + str(beta_term_con) 
										   #+ "\n dissip_term : " + str(-dissip_term) 
										   #+ "\n Y*self.theta_hat : " + str(Y*self.theta_hat)) 

			else:				
				v_des = (k_col_height*ki_col*beta_term_col + k_con_height*ki_con*beta_term_con)
				v_des_dot = (k_col_height*ki_col*beta_term_col_dot + k_con_height*ki_con*beta_term_con_dot)

				e_v = v[self.agent_number] - v_des
				integrator_v = integrator_v + e_v*dt

				ki = 10
				kp = 1
				dissip_term = np.array([k_hor*(kp*kp_vx*e_v[0] + ki*ki_vx*integrator_v[0]), k_hor*(kp*kp_vy*e_v[1] + ki*ki_vy*integrator_v[1]), kp_vz*e_v[2] + ki_vz*integrator_v[2]])
				
				e_tilde = np.zeros(3)

				Y = v_des_dot + [0, 0, grav]

				control = k_col_height*beta_term_col + k_con_height*beta_term_con - dissip_term + 0*Y*self.theta_hat - k_e_tilde*e_tilde  #- np.sign(e_v)*np.linalg.norm(v[0],1)*self.f_b_hat - np.sign(e_v)*self.d_b_hat
				#print(self.topic + " beta_term_col = " + str(beta_term_col))
				#print(self.topic + " beta_term_con = " + str(beta_term_con))
				#print(self.topic + " v_des = " + str(v_des))
				#control = beta_term_col + b=eta_term_con - dissip_term + Y*self.theta_hat - k_e_tilde*e_tilde #- np.sign(e_v)*np.linalg.norm(v[self.agent_number],1)*self.f_b_hat
				#rospy.loginfo("Control of " + self.topic + ": " + str(control) 
										   #+ "\n v_des : " + str(v_des)
										   #+ "\n beta_term_con : " + str(beta_term_con) 
										   #+ "\n dissip_term : " + str(-dissip_term) 
										   #+ "\n Y*self.theta_hat : " + str(Y*self.theta_hat)) 

			# Calculate discrepance terms:
			self.d_b_hat_dot = k_d_b*np.linalg.norm(e_v)
			self.f_b_hat_dot = k_f_b*np.linalg.norm(e_v,1)*np.linalg.norm(v[self.agent_number])
			self.theta_hat_dot = -k_theta*np.dot(Y,e_v)

			mesage_to_pub = mav_msgs.msg.TorqueThrust()
			mesage_to_pub.thrust.x = control[0]
			mesage_to_pub.thrust.y = control[1]
			mesage_to_pub.thrust.z = control[2]

			if dataStore:
				mesage_to_pub.header.frame_id = str(self.agent_number)
				mesage_to_pub.header.stamp = rospy.Time.now()
			self.force_pub.publish(mesage_to_pub)

			if dataStore:
				mesage_beta = PoseStamped()
				mesage_beta.pose.position.x = k_col_height * beta_term_con[0]
				mesage_beta.pose.position.y = k_col_height * beta_term_con[1]
				mesage_beta.pose.position.z = k_col_height * beta_term_con[2]
				mesage_beta.pose.orientation.x = k_col_height*beta_term_col[0]
				mesage_beta.pose.orientation.y = k_col_height*beta_term_col[1]
				mesage_beta.pose.orientation.z = k_col_height*beta_term_col[2]
				mesage_beta.header.frame_id = str(self.agent_number)
				mesage_beta.header.stamp = rospy.Time.now()
				self.beta_pub.publish(mesage_beta)


			# if self.time_store is None:
			# 	self.time_store = np.array([rospy.get_rostime().to_time()]) 		# scalar
			# 	self.pos_store = np.array([[x[self.agent_number]]])					# 3d vector
			# 	self.beta_con_store = np.array([[beta_term_con]])			# 3d vector
			# 	self.beta_col_store = np.array([[beta_term_col]])			# 3d vector
			# 	self.control_store = np.array([[control]])				# 3d vector
			# 	self.dissip_term_store = np.array([[dissip_term]])			# 3d vector
			# 	self.ep_store = np.array([[ep]])					# 3d vector
			# 	self.e_tilde_store = np.array([[e_tilde]])				# 3d vector
			# 	self.e_v_store = np.array([[e_v]])					# 3d vector
			# 	self.v_des_store = np.array([[v_des]])				# 3d vector
			# 	self.v_des_dot_store = np.array([[v_des_dot]])			# 3d vector

			# else: #cancatenate for arrays, append for scalars
			# 	self.time_store = np.append(self.time_store, np.array([rospy.get_rostime().to_time()]))
			# 	self.pos_store = np.concatenate((self.pos_store, np.array([[x[self.agent_number]]])), axis=1)
			# 	self.beta_con_store = np.concatenate((self.beta_con_store, np.array([[beta_term_con]])), axis=1)
			# 	self.beta_col_store = np.concatenate((self.beta_col_store, np.array([[beta_term_col]])), axis=1)
			# 	self.control_store = np.concatenate((self.control_store, np.array([[control]])), axis=1)
			# 	self.dissip_term_store = np.concatenate((self.dissip_term_store, np.array([[dissip_term]])), axis=1)
			# 	self.ep_store = np.concatenate((self.ep_store, np.array([[ep]])), axis=1)
			# 	self.e_tilde_store = np.concatenate((self.e_tilde_store, np.array([[e_tilde_store]])), axis=1)
			# 	self.e_v_store = np.concatenate((self.e_v_store, np.array([[e_v]])), axis=1)
			# 	self.v_des_store = np.concatenate((self.v_des_store, np.array([[v_des]])), axis=1)
			# 	self.v_des_dot_store = np.concatenate((self.v_des_dot_store, np.array([[v_des_dot]])), axis=1)

			self.rate.sleep()

	# def save_data(self):
	# 	np.save("../CrazyDrones/src/multi_crazyflie_simulator/simulation_data/try_1/time" + self.topic + "_time", self.time_store)
	# 	np.save("../CrazyDrones/src/multi_crazyflie_simulator/simulation_data/try_1/position" + self.topic + "_pos", self.pos_store)
	# 	np.save("../CrazyDrones/src/multi_crazyflie_simulator/simulation_data/try_1/beta_term_con" + self.topic + "_beta_con", self.beta_con_store)
	# 	np.save("../CrazyDrones/src/multi_crazyflie_simulator/simulation_data/try_1/beta_term_col" + self.topic + "_beta_col", self.beta_col_store)
	# 	np.save("../CrazyDrones/src/multi_crazyflie_simulator/simulation_data/try_1/control" + self.topic + "_control", self.control_store)
	# 	np.save("../CrazyDrones/src/multi_crazyflie_simulator/simulation_data/try_1/dissip_term" + self.topic + "_dissip_term", self.dissip_term_store)
	# 	np.save("../CrazyDrones/src/multi_crazyflie_simulator/simulation_data/try_1/ep" + self.topic + "_ep", self.ep_store)
	# 	np.save("../CrazyDrones/src/multi_crazyflie_simulator/simulation_data/try_1/e_tilde" + self.topic + "_e_tilde", self.e_tilde_store)
	# 	np.save("../CrazyDrones/src/multi_crazyflie_simulator/simulation_data/try_1/e_v" + self.topic + "_e_v", self.e_v_store)
	# 	np.save("../CrazyDrones/src/multi_crazyflie_simulator/simulation_data/try_1/v_des" + self.topic + "_v_des", self.v_des_store)
	# 	np.save("../CrazyDrones/src/multi_crazyflie_simulator/simulation_data/try_1/v_des_dot" + self.topic + "_v_des_dot", self.v_des_dot_store)

if __name__ == '__main__':
	control = Nav_control()

	control.navigation()
	rospy.spin()
