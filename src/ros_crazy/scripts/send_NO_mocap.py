#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped
from crazyflie_driver.srv import UpdateParams

def resetKalman():
    
    
    rospy.set_param("kalman/resetEstimation", 1)
    update_params(["kalman/resetEstimation"])
    rospy.sleep(0.1)
    rospy.set_param("kalman/resetEstimation", 0)
    update_params(["kalman/resetEstimation"])

if __name__ == '__main__':
    global msg
    global pub

    rospy.init_node('send_mocap_external_positioning', anonymous=True)
    #topic = rospy.get_param("~topic")
    
    rospy.loginfo("CREATED NODE: end_mocap_external_positioning")

    # Allow communication with the crazyflie
    rospy.wait_for_service('update_params')
    rospy.loginfo("found update_params service")
    update_params = rospy.ServiceProxy('update_params', UpdateParams)

    rospy.loginfo("INIT PARAM")

    # Message type
    msg = PointStamped()
    msg.header.seq = 0
    msg.header.stamp = rospy.Time.now()

    # Will send the external position of type PointStamped to the Crazyflie
    pub = rospy.Publisher("external_position", PointStamped, queue_size=1)
    rospy.loginfo("PUBLISHED to external position")

    # Me subscribo a "topic" para recibir un mensaje del tipo Odometry y cuando
    # lo reciba, llamo a onNewPosition
    #rospy.Subscriber("/"+topic, Odometry, onNewPosition)
    #rospy.loginfo("SUBSCRIBED to mocap info")

    resetKalman()
	
    rate = rospy.Rate(100) # 10hz
    # Impide que se acabe el programa
    while not rospy.is_shutdown():
        msg.header.frame_id = ""
        msg.header.seq += 1
        msg.point.x = 0.9
        msg.point.y = 0.5
        msg.point.z = 0
        pub.publish(msg)
        rate.sleep()
