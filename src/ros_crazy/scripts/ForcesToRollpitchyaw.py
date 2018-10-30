#!/usr/bin/env python

import numpy as np
import rospy
import nav_msgs.msg
import mav_msgs.msg
import geometry_msgs.msg
import utilities.utility_functions as utility_functions

# desired_3d_force : input
# psi_angle: yaw angle

def getForces(force_msg):
    desired_3d_force = force_msg


topic = rospy.get_param("~topic")


rospy.init_node("force_to_att", anonymous = True)
sub = rospy.Subscriber("force_from_terminal", Twist, getForces)
pub = rospy.Publisher(topic + "/cmd_vel", Twist, queue_size=1) ### Check type of message!!!

norm = np.linalg.norm(desired_3d_force)
if norm > 0.1:        # could change threshold 0.1
    n_des     = desired_3d_force/norm
    n_des_rot = utility_functions.rot_z(-psi_angle).dot(n_des)
else:
    n_des     = np.array([0.0,0.0,1.0])
    n_des_rot = utility_functions.rot_z(-psi_angle).dot(n_des)

sin_phi   = -n_des_rot[1]
sin_phi   = np.clip(sin_phi,-1,1)
phi       = np.arcsin(sin_phi)

sin_theta = n_des_rot[0]/np.cos(phi)
sin_theta = np.clip(sin_theta,-1,1)
cos_theta = n_des_rot[2]/np.cos(phi)
cos_theta = np.clip(cos_theta,-1,1)
pitch     = np.arctan2(sin_theta,cos_theta)
   
roll_desired = roll
pitch_desired = pitch

message  = mav_msgs.msg.RollPitchYawrateThrust()
message.header.stamp = rospy.get_rostime()
message.roll     = roll_desired
message.pitch    = pitch_desired
message.yaw_rate = yaw_rate_desired
message.thrust.x = 0
message.thrust.y = 0
message.thrust.z = desired_3d_force[2]

pub.Publish(message)