#!/usr/bin/env python

import numpy as np
from numpy import linalg 
import rospy
import std_msgs.msg
import nav_msgs.msg
import geometry_msgs.msg
import rosgraph_msgs.msg
import std_srvs.srv
import tf.transformations
from crazyflie_driver.msg import Position

#import yaw_controller.yaw_controller as yaw_controller

class Nav_control():

	def __init__(self):

		rospy.init_node('swarm_control_pos') 
		
		self.agent_pose= np.array([nav_msgs.msg.Odometry(),nav_msgs.msg.Odometry(),nav_msgs.msg.Odometry(),nav_msgs.msg.Odometry(),nav_msgs.msg.Odometry(),nav_msgs.msg.Odometry()])
		
		self.topic = rospy.get_param("~topic")
		self.agent_number = rospy.get_param("~agent_number")
		self.priority = rospy.get_param("~priority")
		self.num_cf = 2

		# PUBLISHER
		#if self.agent_number != 0:
		self.force_pub = rospy.Publisher(self.topic + "/cmd_pos", Position, queue_size = 100)

		time = rospy.get_time()

		# Trayectory to follow:
		#self.PoI = np.array([[0, 0, 5], [4, 5, 3],[-2, 4, 2],[3, -2, 3]])
		self.PoI = np.array([[2, 2, 1], [-2, 2, 1],[-2, -2, 1],[2, -2, 1]])
		#self.PoI = np.array([[0, 0, 1], [0, 0, 3],[0, 0,5],[5, 0, 5]])
		#self.PoI = np.array([[0, 0, 1], [0, 0, 2],[0, 0,3],[0, 0, 4]])

		self.region_idx = 0
		
		self.r = 0.075

		self.rate = rospy.Rate(10) 

				# SUBSCRIBER
		rospy.Subscriber("/crazyflie_0/out_pos_odometry", nav_msgs.msg.Odometry, self.callback_crazyflie_0)
		# rospy.Subscriber("/crazyflie_1/out_pos_odometry", nav_msgs.msg.Odometry, self.callback_crazyflie_1)
		# rospy.Subscriber("/crazyflie_2/out_pos_odometry", nav_msgs.msg.Odometry, self.callback_crazyflie_2)
		# rospy.Subscriber("/crazyflie_3/out_pos_odometry", nav_msgs.msg.Odometry, self.callback_crazyflie_3)
		# rospy.Subscriber("/crazyflie_4/out_pos_odometry", nav_msgs.msg.Odometry, self.callback_crazyflie_4)
		# rospy.Subscriber("/crazyflie_5/out_pos_odometry", nav_msgs.msg.Odometry, self.callback_crazyflie_5)


	def position_from_odometry(self, odometry):
		x = np.zeros(3)
		x[0] = odometry.pose.pose.position.x
		x[1] = odometry.pose.pose.position.y
		x[2] = odometry.pose.pose.position.z

		return x

	### Remap to change the number os the Crazyflie we refer depending on which Crazyflie runs the code:
	#		This remap is donne following the graph that is already given
	def callback_crazyflie_0(self, data):
		if self.agent_number == 0:
			self.agent_pose[0] = data
		else: 
			self.agent_pose[1] = data

	# def callback_crazyflie_1(self, data): 
	# 	if self.agent_number == 0: 
	# 		self.agent_pose[1] = data
	# 	elif self.agent_number == 1:
	# 		self.agent_pose[0] = data
	# 	else: 
	# 		self.agent_pose[2] = data 

	# def callback_crazyflie_2(self, data):  
	# 	if self.agent_number == 0 or self.agent_number == 1: 
	# 		self.agent_pose[2] = data
	# 	elif self.agent_number == 2:
	# 		self.agent_pose[0] = data
	# 	else: 
	# 		self.agent_pose[3] = data 

	# def callback_crazyflie_3(self, data):  
	# 	if self.agent_number == 4 or self.agent_number == 5: 
	# 		self.agent_pose[4] = data
	# 	elif self.agent_number == 3:
	# 		self.agent_pose[0] = data
	# 	else: 
	# 		self.agent_pose[3] = data 

	# def callback_crazyflie_4(self, data):  
	# 	if self.agent_number == 5: 
	# 		self.agent_pose[5] = data
	# 	elif self.agent_number == 4:
	# 		self.agent_pose[0] = data
	# 	else: 
	# 		self.agent_pose[4] = data 

	# def callback_crazyflie_5(self, data): 
	# 	if self.agent_number == 5:
	# 		self.agent_pose[0] = data
	# 	else: 
	# 		self.agent_pose[5] = data 

	
	def conections(self, distancia):

		dist = np.linalg.norm(distancia)
		a = (0, 0, 0)
		if 1.2 < dist and dist < 3:
			a = (dist - 1.2) * distancia * 1/dist
		
		elif  dist > 3:
			a = (0,0,0) 
		return a

	def collision(self, distancia):

		dist = np.linalg.norm(distancia)
		b = (0, 0, 0)
		if 0.5 < dist and dist < 1.2:
			b = -(1.2 - dist) *  distancia * 1/dist

		elif  dist < 0:
			b = (0,0,0) 
		return b 

	def navigation(self):


		mass = 0.027
		dt = 0.01
		
		mode = 0
		second_reg = False
		grav = 9.81

		x = [np.zeros(3),np.zeros(3),np.zeros(3),np.zeros(3),np.zeros(3),np.zeros(3)]
		

		while not rospy.is_shutdown():
			
			for i in range(0, 1):

				x[i] = self.position_from_odometry(self.agent_pose[i])
			
			# Check if we are lider o follower
			if self.priority == 1: # Lider
				mode = 1 
			else:                  # Follower
				mode = 0

			# Point to achieve
			xd = self.PoI[self.region_idx]


			if mode == 1:

				x_desired = xd
				ep = x[0] - xd

				if np.linalg.norm(ep) < 0.05:	# Cheqck if desired point is achieved
					
					if self.region_idx == 3: 
						print ('REACHED!!! POINT NUMBER ' + str(self.region_idx))
						self.region_idx = 0
					if self.region_idx < 3:
						print ('REACHED!!! POINT NUMBER ' + str(self.region_idx))
						self.region_idx+= 1			# Another point to achieve
					

			# Define the conections in the graph
			# Edges = {(0,1), (0,2), (0,3), (2,3), (2,4), (4,5), (5,1)}

			# Definition of etas depending on the Crazyflie we are
	
				
			if self.agent_number == 1:
				distancia_0 = x[1] - x[0]
				distancia_1 = x[2] - x[0]
				x_desired = x[0] + self.conections(distancia_0) - self.collision(distancia_1)


			# if self.agent_number == 2:
			#  	distancia_0 = x[1] - x[0]
			# 	distancia_1 = x[2] - x[0]
			# 	x_desired = x[0] + self.conections(distancia_0) - self.collision(distancia_1)
				

			# if self.agent_number == 3:
			# 	eta_con[0], eta_con_dot[0] = self.eta_funtion(x[0], x[1], v[0], v[1])
			# 	eta_con[1], eta_con_dot[1] = self.eta_funtion(x[0], x[3], v[0], v[3])

			# if self.agent_number == 4:
			# 	eta_con[0], eta_con_dot[0] = self.eta_funtion(x[0], x[3], v[0], v[3])
			# 	eta_con[1], eta_con_dot[1] = self.eta_funtion(x[0], x[5], v[0], v[5])

			# if self.agent_number == 5:
			# 	eta_con[0], eta_con_dot[0] = self.eta_funtion(x[0], x[5], v[0], v[5])
			# 	eta_con[1], eta_con_dot[1] = self.eta_funtion(x[0], x[2], v[0], v[2])


			mesage_to_pub = Position()
			mesage_to_pub.x = x_desired[0]
			mesage_to_pub.y = x_desired[1]
			mesage_to_pub.z = x_desired[2]
			self.force_pub.publish(mesage_to_pub)

			self.rate.sleep()

if __name__ == '__main__':
    control = Nav_control()
    
    control.navigation()
    rospy.spin()
