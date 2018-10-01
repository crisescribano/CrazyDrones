#!/usr/bin/env python

import numpy as np
import rospy
import sys

import std_srvs.srv
import nav_msgs.msg
import tf.transformations

import geometry_msgs.msg
import crazyflie_driver.msg
import crazyflie_driver.srv

class SetpointBroadcaster:

    def __init__(self):


        rospy.init_node('drone_controller')

        rospy.wait_for_service('update_params')
        rospy.loginfo("found update_params service")
        self.update_params = rospy.ServiceProxy('update_params', crazyflie_driver.srv.UpdateParams)

        self.pub_external_pos = rospy.Publisher('external_position', geometry_msgs.msg.PointStamped, queue_size=10)
        self.pub_cmd_pos = rospy.Publisher('cmd_position', crazyflie_driver.msg.Position, queue_size=10)
        self.sub_mocap = rospy.Subscriber('odometry', nav_msgs.msg.Odometry, self.get_mocap)
        self.sub_waypoint = rospy.Subscriber('command/pose', geometry_msgs.msg.PoseStamped, self.get_target_state)
        self.rate = rospy.Rate(50)
        self.mode_srv = rospy.Service('SetFlight', std_srvs.srv.SetBool, self.set_flight_mode)


        # Initialize variables
        self.position = np.array([0,0,0])
        self.attitude = np.array([0,0,0])
        self.desired_position = np.array([0,0,0])
        self.desired_attitude = np.array([0,0,0])
        self.first_transform = True

        # Safety Variable for Mocap - counter for lost mocap frames
        self.invalid_mocap_counter = 0
        self.INVALID_MOCAP_THRESH = 30

        # Initialize flightmode - start controller in 'Stop'-mode
        self.flightmode = 0



    def get_mocap(self, msg = nav_msgs.msg.Odometry()):
        self.mocap_timestamp = msg.header.stamp
        self.position = np.array([msg.pose.pose.position.x , \
                                  msg.pose.pose.position.y , \
                                  msg.pose.pose.position.z])

        quat = np.array([msg.pose.pose.orientation.x, \
                         msg.pose.pose.orientation.y, \
                         msg.pose.pose.orientation.z, \
                         msg.pose.pose.orientation.w])

        self.attitude = tf.transformations.euler_from_quaternion(quat, axes='sxyz')


        if self.first_transform:
            # initialize kalman filter
            rospy.set_param("kalman/initialX", float(self.position[0]))
            rospy.set_param("kalman/initialY", float(self.position[1]))
            rospy.set_param("kalman/initialZ", float(self.position[2]))
            self.update_params(["kalman/initialX", "kalman/initialY", "kalman/initialZ"])

            rospy.set_param("kalman/resetEstimation", 1)
            self.update_params(["kalman/resetEstimation"])
            rospy.set_param("kalman/resetEstimation", 0)
            self.update_params(["kalman/resetEstimation"])
            rospy.set_param("kalman/resetEstimation", 1)
            self.update_params(["kalman/resetEstimation"])
            self.first_transform = False
            rospy.loginfo("Updated Kalman Fitler and resetted the estimator.")
        else:
            external_pos = self.getExternalPosition()
            self.pub_external_pos.publish(external_pos)

    def get_target_state(self, msg = geometry_msgs.msg.PoseStamped()):

        self.desired_position = np.array([msg.pose.position.x, \
                                          msg.pose.position.y, \
                                          msg.pose.position.z ])

        quat = np.array([msg.pose.orientation.x, \
                        msg.pose.orientation.y, \
                        msg.pose.orientation.z, \
                        msg.pose.orientation.w])

        self.desired_attitude = tf.transformations.euler_from_quaternion(quat, axes='sxyz')


    def set_flight_mode(self, req = std_srvs.srv.SetBoolRequest() ):
        rsp = std_srvs.srv.SetBoolResponse()

        # Parse request
        if int(req.data) != self.flightmode:
            self.flightmode = int(req.data)
            rsp.success = True
            if req.data == True:
                rsp.message = "Going into flight..."
                self.desired_position = self.position
                self.desired_attitude = self.attitude
            else:
                rsp.message = "Landing UAV"
        else:
            rsp.success = False
            rsp.message = "Already at desired state"

        return rsp

    def getExternalPosition(self):

        external_pos = geometry_msgs.msg.PointStamped()
        external_pos.point.x = self.position[0]
        external_pos.point.y = self.position[1]
        external_pos.point.z = self.position[2] 

        return external_pos

    def getDesiredPosition(self):

        desired_pos = crazyflie_driver.msg.Position()
        desired_pos.x = self.desired_position[0]
        desired_pos.y = self.desired_position[1]
        desired_pos.z = self.desired_position[2]

        return desired_pos


    def run(self):

        # Loop while ROS node is active
        while not rospy.is_shutdown():
            # Only apply control if the flightmode is on
            if self.flightmode:
                desired_pos = self.getDesiredPosition()                
                self.pub_cmd_pos.publish(desired_pos)

            self.rate.sleep()

        return


if __name__ == '__main__':
    controller = SetpointBroadcaster()
    controller.run()