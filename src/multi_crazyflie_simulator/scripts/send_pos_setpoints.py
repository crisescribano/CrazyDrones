#!/usr/bin/env python

import numpy as np
import rospy
import tf

from crazyflie_driver.msg import Position
from geometry_msgs.msg import Twist

#rostopic pub /<nombre_del_node_crazyflie>/pos_from_terminal geometry_msgs/Twist  '[linear:  {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}]'
#rostopic pub /crazyflie/pos_from_terminal geometry_msgs/Twist  '[0.0, 0.0, 0.5]' '[0.0, 0.0, 0.0]'

class SendSetpoint(): 

	def __init__(self):

		self.started = False
		self.pos_setpoint = [0.0, 0.0, 0.0, 0.0]
		rospy.init_node("send_pos_setpoints", anonymous=True)

		#self.worldFrame = rospy.get_param("~worldFrame", "/world")

		self.msg = Position()
		self.pub = rospy.Publisher("/crazyflie_0/cmd_pos", Position, queue_size=1)
		self.sub = rospy.Subscriber("pos_from_terminal", Twist, self.getPose)
		self.rate = rospy.Rate(10)
		
	def getPose(self, messageIn):
		print("MESSAGE IN: pos to send: " + str(self.pos_setpoint[0]))
		self.pos_setpoint = [messageIn.linear.x, messageIn.linear.y, messageIn.linear.z, messageIn.angular.z]
		if not self.started:
			self.started = True

	def _main_loop(self):

		while(not rospy.is_shutdown()): 
			if(self.started):
				self.msg.header.seq = 0
				self.msg.header.stamp = rospy.Time.now()
				#self.msg.header.frame_id = self.worldFrame
				self.msg.x = self.pos_setpoint[0]
				self.msg.y = self.pos_setpoint[1]
				self.msg.z = self.pos_setpoint[2]
				self.msg.yaw = self.pos_setpoint[3]
				self.pub.publish(self.msg)
				self.rate.sleep()

if __name__ == '__main__':
	sendSetpoint = SendSetpoint()
	sendSetpoint._main_loop()
