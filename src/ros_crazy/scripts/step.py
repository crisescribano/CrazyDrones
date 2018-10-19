#!/usr/bin/env python

import rospy
import numpy as np
from math import cos, sin, tan
from geometry_msgs.msg import PointStamped

from cf_physical_parameters import CF_parameters
from cf_pid_params import CF_pid_params
from pid import PID
from crazyflie_driver.msg import Position
from geometry_msgs.msg import Twist



class CF_state():

    def __init__(self):
        self.position = np.zeros(3)
        self.lin_vel = np.zeros(3)
        self.attitude = np.zeros(3)
        self.ang_vel = np.zeros(3)   
        self.motor_pwm = np.zeros(CF_parameters().NUM_MOTORS)
        self.motor_rotation_speed = np.zeros(CF_parameters().NUM_MOTORS)
        self.sum_motor_rotations = 0.0
        self.forces = np.zeros(3)
        self.momentums = np.zeros(3)

    def getMotorRotationSpeed(self):
        for i in range(0,4):
            self.motor_rotation_speed[i] = 0.2685*self.motor_pwm[i] + 4070.3

    def addMotorsRotationsSpeed(self):
        for i in range(0,4):
            self.sum_motor_rotations = self.sum_motor_rotations + self.motor_rotation_speed[i]

    def getForces(self):
        self.forces = np.array([0, 0, CF_parameters().KT*self.sum_motor_rotations])

    def getMomentums(self):
        self.momentums[0] = (CF_parameters().L * CF_parameters().KT/np.sqrt(2))*(-self.motor_rotation_speed[0]**2 - self.motor_rotation_speed[1]**2 + self.motor_rotation_speed[2]**2 + self.motor_rotation_speed[3]**2)
        self.momentums[1] = (CF_parameters().L * CF_parameters().KT/np.sqrt(2))*(-self.motor_rotation_speed[0]**2 + self.motor_rotation_speed[1]**2 + self.motor_rotation_speed[2]**2 - self.motor_rotation_speed[3]**2)
        self.momentums[2] = CF_parameters().KD * (-self.motor_rotation_speed[0]**2 + self.motor_rotation_speed[1]**2 - self.motor_rotation_speed[2]**2 + self.motor_rotation_speed[3]**2)


class CF_model():

    def __init__(self):

        rospy.init_node("step", anonymous=True)
        self.pub = rospy.Publisher("state_estimation", Position, queue_size=1)
        #rospy.Subscriber("cmd_vel", Twist, self.NewInfoAttitude)
        self.rate = rospy.Rate(50)
        self.msg = Position()


        # System state: position, linear velocities,
        # attitude and angular velocities
        self.cf_state = CF_state()

        # Import the crazyflie physical paramters
        #     - These parameters are obtained from different sources.
        #     - For these parameters and the dynamical equations refer
        #       to : DESIGN OF A TRAJECTORY TRACKING CONTROLLER FOR A
        #            NANOQUADCOPTER
        #            Luis, C., & Le Ny, J. (August, 2016)
        self.cf_physical_params = CF_parameters()

        # Import the PID gains (from the firmware)
        self.cf_pid_gains = CF_pid_params()

        # Main CF variables initialization (if needed)
        self.simulation_freq = rospy.Rate(int(1/self.cf_physical_params.DT_CF))

        ######################
        # Initialize PID
        ######################

        # Out from the PIDs, values of
        # r, p, y, thrust
        self.desired_rpy = np.zeros(3)

        # Comes from the external position controller
        self.desired_thrust = 0.0

        self.desired_ang_vel = np.zeros(3)

        self.desired_att = np.zeros(3)


    def rotation_matrix(self, roll, pitch, yaw):
        return np.array([[cos(pitch)*cos(yaw), cos(pitch)*sin(yaw), -sin(pitch)],
                            [sin(roll)*sin(pitch)*cos(yaw) - cos(roll)*sin(yaw), sin(roll)*sin(pitch)*sin(yaw) + cos(roll)*cos(yaw), sin(roll)*cos(pitch)],
                            [cos(roll)*sin(pitch)*cos(yaw) + sin(roll)*sin(yaw), cos(roll)*sin(pitch)*sin(yaw) - sin(roll)*cos(yaw),cos(roll)*cos(pitch)]])

    def euler_matrix(self, roll, pitch, yaw):
        if(pitch != np.pi/2):
            return np.array([[1, sin(roll)*tan(pitch), cos(roll)*tan(yaw)],
                            [0, cos(roll), -sin(roll)],
                            [0, sin(roll)/cos(pitch), cos(roll)/cos(pitch)]])
        else:
            return np.array([[1, sin(roll) * tan(pitch), cos(roll) * tan(yaw)],
                             [0, cos(roll), -sin(roll)],
                             [0, sin(roll) / (cos(pitch) + 1e-100), cos(roll) / (cos(pitch) + 1e-100)]])


    def apply_simulation_step(self):


        ###########################
        # Main simulation loop
        #   - The CF works at a rate of 1000Hz,
        #     in the same way, we are simulating
        #     at the same frequency
        ###########################

        # New simulated state
        new_state = CF_state()

        rotation_matrix = self.rotation_matrix(self.cf_state.attitude[0],self.cf_state.attitude[1], self.cf_state.attitude[2])
        euler_matrix = self.euler_matrix(self.cf_state.attitude[0],self.cf_state.attitude[1], self.cf_state.attitude[2])

        #print("Rotation matrix[0]: " + str(rotation_matrix[0][0]) + ", "+ str(rotation_matrix[0][1]) + ", "+ str(rotation_matrix[0][2]))
        #print("Rotation matrix[1]: " + str(rotation_matrix[1][0]) + ", "+ str(rotation_matrix[1][1]) + ", "+ str(rotation_matrix[1][2]))
        #print("Rotation matrix[2]: " + str(rotation_matrix[2][0]) + ", "+ str(rotation_matrix[2][1]) + ", "+ str(rotation_matrix[2][2]))
        

        #print("Euler matrix[0]: " + str(euler_matrix[0][0]) + ", "+ str(euler_matrix[0][1]) + ", "+ str(euler_matrix[0][2]))
        #print("Euler matrix[1]: " + str(euler_matrix[1][0]) + ", "+ str(euler_matrix[1][1]) + ", "+ str(euler_matrix[1][2]))
        #print("Euler matrix[2]: " + str(euler_matrix[2][0]) + ", "+ str(euler_matrix[2][1]) + ", "+ str(euler_matrix[2][2]))
        
        self.cf_state.getMotorRotationSpeed()
        #print("getMotorRotationSpeed: " + str(self.cf_state.motor_rotation_speed[0]) + ", " + str(self.cf_state.motor_rotation_speed[1]) + ", "+ str(self.cf_state.motor_rotation_speed[2]))
        self.cf_state.addMotorsRotationsSpeed()
        #print("addMotorsRotationsSpeed: " + str(self.cf_state.sum_motor_rotations))
        self.cf_state.getForces()
        print("forces: " + str(self.cf_state.forces[0])+ ", "+ str(self.cf_state.forces[1])+ ", "+ str(self.cf_state.forces[2]) )
        self.cf_state.getMomentums()

        #print("Last state: Velocidad lineal[0]: " + str(new_state.lin_vel[0]))
        #print("Last state: Velocidad lineal[1]: " + str(new_state.lin_vel[1]))
        #print("Last state: Velocidad lineal[2]: " + str(new_state.lin_vel[2]))

        new_state.lin_vel = np.array([0, 0, self.cf_state.forces[2]/self.cf_physical_params.DRONE_MASS]) - np.matmul(rotation_matrix, [0, 0, self.cf_physical_params.G]) - np.cross(self.cf_state.ang_vel, self.cf_state.lin_vel)
        #print("After integration. lin_vel[0]: " + str(new_state.lin_vel[0]))
        #print("After integration. lin_vel[1]: " + str(new_state.lin_vel[1]))
        #print("After integration. lin_vel[3]: " + str(new_state.lin_vel[2]))

        #print("New state: Velocidad lineal[0]: " + str(new_state.lin_vel[0]))
        #print("New state: Velocidad lineal[1]: " + str(new_state.lin_vel[1]))
        #print("New state: Velocidad lineal[2]: " + str(new_state.lin_vel[2]))

        new_state.position = np.dot(np.transpose(rotation_matrix), self.cf_state.lin_vel)

        preoperation = self.cf_state.momentums - np.cross(self.cf_state.ang_vel,
                                            np.dot(self.cf_physical_params.INERTIA_MATRIX,
                                            self.cf_state.ang_vel))

        new_state.ang_vel = np.dot(self.cf_physical_params.INV_INERTIA_MATRIX, preoperation)
        
        new_state.attitude = np.dot(euler_matrix, self.cf_state.ang_vel)

        for i in range(0, 3):
            self.cf_state.position[i] = self.cf_state.position[i] + (new_state.position[i] * self.cf_physical_params.DT_CF)
            self.cf_state.attitude[i] = self.cf_state.attitude[i] + (new_state.attitude[i] * self.cf_physical_params.DT_CF)
            self.cf_state.lin_vel[i] = self.cf_state.lin_vel[i] + (new_state.lin_vel[i] * self.cf_physical_params.DT_CF)
            self.cf_state.ang_vel[i] = self.cf_state.ang_vel[i] + (new_state.ang_vel[i] * self.cf_physical_params.DT_CF)

            #print("Nuevo estado: " + str(self.cf_state.position[i]) + ", " + str(self.cf_state.attitude[i]) + ", " + str(self.cf_state.lin_vel[i]) + ", " + str(self.cf_state.ang_vel[i]) + ".")

    def publish_state(self):
       
        #print("MESSAGE IN: cf_state.position[0] " + str(self.cf_state.position[0]))
        #print("MESSAGE IN: cf_state.position[1] " + str(self.cf_state.position[1]))
        #print("MESSAGE IN: cf_state.position[2] " + str(self.cf_state.position[2]))

        self.msg.header.seq = 0
        self.msg.header.stamp = rospy.Time.now()
        self.msg.header.frame_id = "1"
        self.msg.x = self.cf_state.position[0]
        self.msg.y = self.cf_state.position[1]
        self.msg.z = self.cf_state.position[2]
        self.msg.yaw = 0.0
        self.pub.publish(self.msg)
        self.rate.sleep()

    def run(self):
        while(not rospy.is_shutdown()):
            self.apply_simulation_step()

        # Wait for the cycle left time
        self.simulation_freq.sleep()

if __name__ == '__main__':
    model = CF_model()
    
    model.run()
    rospy.spin()