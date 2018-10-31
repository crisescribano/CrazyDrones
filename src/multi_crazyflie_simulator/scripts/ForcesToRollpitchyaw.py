#!/usr/bin/env python

import numpy as np
import rospy
from geometry_msgs.msg import Twist
from crazyflie_driver.msg import GenericLogData

# desired_3d_force: input
# psi_angle: yaw angle

class Forces():

	def __init__(self):

		# Init forces node:
		rospy.init_node("force_to_att", anonymous = True)

		# Comunication:
		#rospy.Subscriber("force_from_terminal", GenericLogData, getForces)
		self.pub = rospy.Publisher("/crazyflie_0/cmd_vel", Twist, queue_size=1)

		self.desired_3d_force = [0, 0, 0]
		self.M = [(0,0,0), (0,0,0),(0,0,0)]
		self.psi_angle = 0 # We do not care about the yaw rate
		self.I = [(1,0,0), (0,1,0),(0,0,1)]

	#def getForces(self, force_msg):
    #	desired_3d_force = [force_msg[0], force_msg[1],force_msg[2]]

	# def rot_z(self, gamma):
 #        cos_g = cos(gamma)
 #        sin_g = sin(gamma)
 #        self.M = np.array([[cos_g, sin_g, 0], [-sin_g, cos_g, 0], [0, 0, 1]])
 #        return self.M

	def run(self):
		while(not rospy.is_shutdown()):
			self.desired_3d_force = [0, 0, 0]
			
			norm = np.linalg.norm(self.desired_3d_force)

			if norm > 0.1:        # could change threshold 0.1
				n_des = self.desired_3d_force/norm
				#preop = self.rot_z(-self.psi_angle)
				n_des_rot = np.dot(self.I, n_des)
			else:
				n_des = np.array([0.0,0.0,1.0])
				#preop = self.rot_z(-self.psi_angle)
				n_des_rot = np.dot(self.I, n_des)

			sin_phi = -n_des_rot[1]
			sin_phi = np.clip(sin_phi,-1,1)
			roll = np.arcsin(sin_phi)
			phi = roll

			sin_theta = n_des_rot[0]/np.cos(phi)
			sin_theta = np.clip(sin_theta,-1,1)
			cos_theta = n_des_rot[2]/np.cos(phi)
			cos_theta = np.clip(cos_theta,-1,1)
			pitch = np.arctan2(sin_theta,cos_theta)


			message = Twist()
			#message.header.stamp = rospy.get_rostime()
			message.linear.x = roll 
			message.linear.y = pitch
			#message.angular.z = yaw_rate_desired
			message.linear.z = self.desired_3d_force[2]

			self.pub.publish(message)

if __name__ == '__main__':
	forces = Forces()

	forces.run()
	rospy.spin()