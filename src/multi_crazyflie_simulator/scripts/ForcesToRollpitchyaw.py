#!/usr/bin/env python

import numpy as np
import rospy
from geometry_msgs.msg import Twist
import mav_msgs.msg
from cf_physical_parameters import CF_parameters
import time

# desired_3d_force: input
# psi_angle: yaw angle

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
		#print("message publish forces: "+ str(force_msg.thrust.x) + ", "+ str(force_msg.thrust.y) + ", "+ str( force_msg.thrust.z))
		self.desired_3d_force[0] = force_msg.thrust.x
		self.desired_3d_force[1] = force_msg.thrust.y
		self.desired_3d_force[2] = force_msg.thrust.z

		#print("Forces file")

		norm = np.linalg.norm(self.desired_3d_force)

		n_des = self.desired_3d_force/(norm + 1e-12)
		# #preop = self.rot_z(-self.psi_angle)
		# n_des_rot = np.dot(self.I, n_des)
		# print(n_des)
		# sin_phi = -n_des_rot[1]
		# sin_phi = np.clip(sin_phi,-1,1)
		# roll = np.arcsin(sin_phi)
		# phi = roll

		# sin_theta = n_des_rot[0]/np.cos(phi)
		# sin_theta = np.clip(sin_theta,-1,1)
		# cos_theta = n_des_rot[2]/np.cos(phi)
		# cos_theta = np.clip(cos_theta,-1,1)
		# pitch = np.arctan2(sin_theta,cos_theta)
		pitch = np.arcsin(n_des[0])
		roll = -np.arcsin(n_des[1])

		#rospy.loginfo("desired force for " + str(self.topic) + " = " + str(self.desired_3d_force))
		#print("roll, pitch = ", roll, pitch)
		#time.sleep(0.01)
		

		message = Twist()
		#message.header.stamp = rospy.get_rostime()
		message.linear.x = pitch*180/np.pi
		#print(message.linear.x)

		message.linear.y = roll*180/np.pi
		#message.angular.z = yaw_rate_desired
		raw_thrust = np.sign(self.desired_3d_force[2])*(np.sqrt(np.absolute(self.desired_3d_force[2])/self.cf_physical_params.CT) - 4070.3)/0.2685
		
		message.linear.z = max((raw_thrust + self.cf_physical_params.BASE_THRUST), self.cf_physical_params.PWM_MIN)
		#rospy.loginfo("Forces sent: " + str(message.linear.z))
		#print("message publish in cmd_vel: "+ str(message.linear.x) + ", "+ str(message.linear.y) + ", "+ str(message.linear.z))
		#print("desired forces: "+ str(self.desired_3d_force[0]) + ", "+ str(self.desired_3d_force[1]) + ", "+ str(self.desired_3d_force[2]))
		self.pub.publish(message)

if __name__ == '__main__':
	forces = Forces()

	rospy.spin()

