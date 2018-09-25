#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped
from crazyflie_driver.srv import UpdateParams
from crazyflie_driver.msg import GenericLogData

#En el launch file he creado dos topic llamados "stateEstimation" y "kalmanEstimation" que tienen los datos (stateEstimate.x, .y, .z) y (kalman.stateX, .stateY, .stateZ), que es el estado interno del dron, en decir, la posicion que cree que tiene el dron.

def PrintEstadoCf(donde):
    rospy.loginfo("Position thought by the CF: " + str(donde.values[0]) + ", " + str(donde.values[1]) + ", " + str(donde.values[2]))
	


if __name__ == '__main__':

    rospy.init_node('lee_estado_cf', anonymous=True)

    rospy.loginfo("CREATED NODE: lee_estado_cf")

    rospy.Subscriber("stateRead", GenericLogData, PrintEstadoCf)
    rospy.loginfo("SUBSCRIBED to stateEstimation")
	
    # Impide que se acabe el programa
    rospy.spin()
