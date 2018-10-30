#!/usr/bin/env python

import numpy as np
import rospy
import geometry_msgs.msg
from crazyflie_driver.msg import GenericLogData
import utilities.utility_functions as utility_functions

# desired_3d_force : input
# psi_angle: yaw angle

def getForces(force_msg):
    desired_3d_force = [force_msg[0], force_msg[1],force_msg[2]]

topic = rospy.get_param("~topic")

rospy.init_node("force_to_att", anonymous = True)
sub = rospy.Subscriber("force_from_terminal", GenericLogData, getForces)
pub = rospy.Publisher(topic + "/cmd_vel", Twist, queue_size=1)

norm = np.linalg.norm(desired_3d_force)
if norm > 0.1:        # could change threshold 0.1
    n_des = desired_3d_force/norm
    n_des_rot = utility_functions.rot_z(-psi_angle).dot(n_des)
else:
    n_des = np.array([0.0,0.0,1.0])
    n_des_rot = utility_functions.rot_z(-psi_angle).dot(n_des)

sin_phi = -n_des_rot[1]
sin_phi = np.clip(sin_phi,-1,1)
roll = np.arcsin(sin_phi)

sin_theta = n_des_rot[0]/np.cos(phi)
sin_theta = np.clip(sin_theta,-1,1)
cos_theta = n_des_rot[2]/np.cos(phi)
cos_theta = np.clip(cos_theta,-1,1)
pitch = np.arctan2(sin_theta,cos_theta)


message = Twist()
message.header.stamp = rospy.get_rostime()
message.linear.x = roll 
message.linear.y = pitch
#message.angular.z = yaw_rate_desired
message.linear.z = desired_3d_force[2]

pub.Publish(message)