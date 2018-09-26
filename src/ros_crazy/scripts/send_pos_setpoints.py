#!/usr/bin/env python

import rospy
import tf
from crazyflie_driver.msg import Position
from geometry_msgs.msg import Twist

#rostopic pub /<nombre_del_node_crazyflie>/pos_from_terminal geometry_msgs/Twist  '{linear:  {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}'
def getPose(messageIn):
    msg.header.seq = 0
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = worldFrame
    msg.x = messageIn.linear.x
    msg.y = messageIn.linear.y
    msg.z = messageIn.linear.z
    msg.yaw = messageIn.angular.z
    pub.publish(msg)

if __name__ == '__main__':
    global msg
    global worldFrame
    global pub

    rospy.init_node('send_pos_setpoints', anonymous=True)
    worldFrame = rospy.get_param("~worldFrame", "/world")

    msg = Position()

    pub = rospy.Publisher("cmd_position", Position, queue_size=1)
    sub = rospy.Subscriber("pos_from_terminal", Twist, getPose)
