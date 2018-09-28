#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped
from crazyflie_driver.srv import UpdateParams

def onNewPosition(position):
    global msg
    global pub
    global firstPosition


    rospy.loginfo("Received position: " + str(position.pose.pose.position.x) + ", " + str(position.pose.pose.position.y) + ", " + str(position.pose.pose.position.z))


    if firstPosition:

        # initialize kalman filter
        rospy.set_param("kalman/initialX", transform.transform.translation.x)
        rospy.set_param("kalman/initialY", transform.transform.translation.y)
        rospy.set_param("kalman/initialZ", transform.transform.translation.z)
        update_params(["kalman/initialX", "kalman/initialY", "kalman/initialZ"])
    
        # Resetea el kalman filter
	rospy.set_param("kalman/resetEstimation", 1)

        # Aplica los cambios en el drone
        update_params(["kalman/resetEstimation"]) #, "locSrv/extPosStdDev"])
        firstPosition = False
    else:
	rospy.set_param("kalman/resetEstimation", 0) 
        update_params(["kalman/resetEstimation"])
    	
        msg.header.frame_id = position.header.frame_id
    	msg.header.stamp = position.header.stamp
    	msg.header.seq += 1
    	msg.point.x = position.pose.pose.position.x
    	msg.point.y = position.pose.pose.position.y
    	msg.point.z = position.pose.pose.position.z
    	pub.publish(msg)


if __name__ == '__main__':

    rospy.init_node('send_mocap_external_positioning', anonymous=True)
    topic = rospy.get_param("~topic")
    
    rospy.loginfo("CREATED NODE: end_mocap_external_positioning")

    # Allow communication with the crazyflie
    rospy.wait_for_service('update_params')
    rospy.loginfo("found update_params service")
    update_params = rospy.ServiceProxy('update_params', UpdateParams)

    rospy.loginfo("INIT PARAM")

    firstPosition = True

    # Message type
    msg = PointStamped()
    msg.header.seq = 0
    msg.header.stamp = rospy.Time.now()

    # Will send the external position of type PointStamped to the Crazyflie
    pub = rospy.Publisher("external_position", PointStamped, queue_size=1)
    rospy.loginfo("PUBLISHED to external position")

    # Me subscribo a "topic" para recibir un mensaje del tipo Odometry y cuando
    # lo reciba, llamo a onNewPosition
    rospy.Subscriber("/"+topic, Odometry, onNewPosition)
    rospy.loginfo("SUBSCRIBED to mocap info")

    # Impide que se acabe el programa
    rospy.spin()
