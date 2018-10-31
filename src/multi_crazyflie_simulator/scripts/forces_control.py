#!/usr/bin/env python

import numpy as np
import rospy
from crazyflie_driver.msg import GenericLogData
from crazyflie_driver.msg import Position
from geometry_msgs.msg import Twist


class Control():

	def __init__(self):

		# Init forces node:
		rospy.init_node("forces_control", anonymous = True)

		self.topic = rospy.get_param("~topic")
		# Comunication:
		rospy.Subscriber(self.topic + "/out_pos", Position, self.get_pos)
		self.pub = rospy.Publisher(self.topic + "/forces_input", GenericLogData, queue_size=1)
		self.fuerza = [0, 0, 0]
		self.pos = [0, 0, 0]
		self.pos_des = [0, 0, 1]


		self.message = GenericLogData()
		self.message.values = [0, 0, 0]


	def get_pos(self, position_msg):

		self.pos[0] = position_msg.x
		self.pos[1] = position_msg.y
		self.pos[2] = position_msg.z

	def run(self):
		while(not rospy.is_shutdown()):

			self.fuerza[0] = 12000 * (-self.pos[0] + self.pos_des[0])
			self.fuerza[1] = 12000 * (-self.pos[1] + self.pos_des[1])
			self.fuerza[2] = 12000 * (-self.pos[2] + self.pos_des[2])
			
			self.message.values[0] = self.fuerza[0]
			self.message.values[1] = self.fuerza[1]
			self.message.values[2] = self.fuerza[2]

			self.pub.publish(self.message)

if __name__ == '__main__':
	forces_control = Control()

	forces_control.run()
	rospy.spin()