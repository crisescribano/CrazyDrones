#!/usr/bin/env python

import rospy
from crazyflie_driver.msg import Position
from geometry_msgs.msg import Twist

#rostopic pub /crazyflie/rpyt_from_terminal geometry_msgs/Twist  '[0.0, 0.0, 0.0]' '[0.0, 0.0, 0.0]'

class SendSetpointRPYT(): 

	def __init__(self):

		self.started = False
		rospy.init_node("send_setpointsRPYT", anonymous=True)
		self.pos_setpointRPYT = [0.0, 0.0, 0.0, 0.0]

		self.worldFrame = rospy.get_param("~worldFrame", "/world")

		self.msg = Twist()
		self.pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
		self.sub = rospy.Subscriber("rpyt_from_terminal", Twist, self.getRpyt)
		self.rate = rospy.Rate(50)


	def getRpyt(self, messageIn):
		self.pos_setpointRPYT = [messageIn.angular.x, messageIn.angular.y, messageIn.angular.z, messageIn.linear.z]
		if not self.started:
			self.started = True

	def _main_loop(self):

		while(not rospy.is_shutdown()): 
			if(self.started):
				#self.msg.header.seq = 0
				#self.msg.header.stamp = rospy.Time.now()
				#self.msg.header.frame_id = self.worldFrame
				self.msg.angular.x = self.pos_setpointRPYT[0]
				self.msg.angular.y = self.pos_setpointRPYT[1]
				self.msg.angular.z = self.pos_setpointRPYT[2]
				self.msg.linear.z = self.pos_setpointRPYT[3]
				self.pub.publish(self.msg)
				self.rate.sleep()


if __name__ == '__main__':
	sendSetpointRPYT = SendSetpointRPYT()
	sendSetpointRPYT._main_loop()

	### roslaunch ros_crazy launch_qualisys_with_mocap.launch model_in:=crazyflie_1 uri:=radio://0/90/2M/E7E7E7E7EA
