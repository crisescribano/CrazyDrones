#!/usr/bin/env python

import numpy as np
import rospy
from geometry_msgs.msg import Twist
import mav_msgs.msg
from cf_physical_parameters import CF_parameters
import time

class Forces():

	def __init__(self):

		# Init forces node:
		rospy.init_node("force_to_att", anonymous = True)

		self.topic = rospy.get_param("~topic")
		# Comunication:
		rospy.Subscriber(self.topic + "/forces_input", mav_msgs.msg.TorqueThrust, self.getForces)
		self.pub = rospy.Publisher(self.topic + "/cmd_vel", Twist, queue_size=1)

		self.desired_3d_force = [0, 0, 0]
		self.M = [(0,0,0), (0,0,0),(0,0,0)]
		self.psi_angle = 0 # We do not care about the yaw rate
		self.I = [(1,0,0), (0,1,0),(0,0,1)]

		self.cf_physical_params = CF_parameters();

	def getForces(self, force_msg):

		self.desired_3d_force[0] = force_msg.thrust.x
		self.desired_3d_force[1] = force_msg.thrust.y
		self.desired_3d_force[2] = force_msg.thrust.z

		norm = np.linalg.norm(self.desired_3d_force)

		n_des = self.desired_3d_force/(norm + 1e-12)
		
		pitch = np.arcsin(n_des[0])
		roll = -np.arcsin(n_des[1])

		message = Twist()
		message.linear.x = pitch*180/np.pi
		message.linear.y = roll*180/np.pi
		raw_thrust = np.sign(self.desired_3d_force[2])*(np.sqrt(np.absolute(self.desired_3d_force[2])/self.cf_physical_params.CT) - 4070.3)/0.2685
		
		message.linear.z = max((raw_thrust + self.cf_physical_params.BASE_THRUST), self.cf_physical_params.PWM_MIN)
		self.pub.publish(message)

if __name__ == '__main__':
	forces = Forces()

	rospy.spin()

