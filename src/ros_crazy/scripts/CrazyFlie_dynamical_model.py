#!/usr/bin/env python

import rospy
import numpy as np
from math import cos, sin, tan

from cf_physical_parameters import CF_parameters
from cf_pid_params import CF_pid_params
from pid import PID
from crazyflie_driver.msg import Position
from crazyflie_driver.msg import Hover


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

        rospy.init_node("dynamic_model", anonymous=True)
        self.pub = rospy.Publisher("state_estimation", Position, queue_size=1)
        rospy.Subscriber("cmd_hover", Hover, self.NewInfoHover)

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


        ######################
        # Angular velocities
        ######################

        self.wx_pid = PID(self.cf_pid_gains.KP_WX,
                          self.cf_pid_gains.KI_WX,
                          self.cf_pid_gains.KD_WX,
                          self.cf_pid_gains.INT_MAX_WX,
                          self.cf_pid_gains.WX_DT)

        self.wy_pid = PID(self.cf_pid_gains.KP_WY,
                          self.cf_pid_gains.KI_WY,
                          self.cf_pid_gains.KD_WY,
                          self.cf_pid_gains.INT_MAX_WY,
                          self.cf_pid_gains.WY_DT)

        self.wz_pid = PID(self.cf_pid_gains.KP_WZ,
                          self.cf_pid_gains.KI_WZ,
                          self.cf_pid_gains.KD_WZ,
                          self.cf_pid_gains.INT_MAX_WZ,
                          self.cf_pid_gains.WZ_DT)

        self.desired_ang_vel = np.zeros(3)

        ######################                                                  
        # Attitudes
        ######################

        self.roll_pid = PID(self.cf_pid_gains.KP_ROLL,
                          self.cf_pid_gains.KI_ROLL,
                          self.cf_pid_gains.KD_ROLL,
                          self.cf_pid_gains.INT_MAX_ROLL,
                          self.cf_pid_gains.ROLL_DT)

        self.pitch_pid = PID(self.cf_pid_gains.KP_PITCH,
                          self.cf_pid_gains.KI_PITCH,
                          self.cf_pid_gains.KD_PITCH,
                          self.cf_pid_gains.INT_MAX_PITCH,
                          self.cf_pid_gains.PITCH_DT)

        self.att_pid_counter = 0
        self.att_pid_counter_max = int( self.cf_physical_params.DT_ATT_PIDS / self.cf_physical_params.DT_CF) - 1

        self.desired_att = np.zeros(3)

        ############################
        # Communication control
        ############################
        self.out_pos_counter = 0
        self.out_pos_counter_max = int(self.cf_physical_params.DT_COMMUNICATION / self.cf_physical_params.DT_CF) - 1

     
    ###########################
    # Math operations functions
    ###########################

    def rotation_matrix(self, roll, pitch, yaw):
        return np.array([[cos(pitch)*cos(yaw),
                             cos(pitch)*sin(yaw),
                             -sin(pitch)],
                            [sin(roll)*sin(pitch)*cos(yaw) - cos(roll)*sin(yaw),
                             sin(roll)*sin(pitch)*sin(yaw) + cos(roll)*cos(yaw),
                             sin(roll)*cos(pitch)],
                            [cos(roll)*sin(pitch)*cos(yaw) + sin(roll)*sin(yaw),
                             cos(roll)*sin(pitch)*sin(yaw) - sin(roll)*cos(yaw),
                             cos(roll)*cos(pitch)]])

    def euler_matrix(self, roll, pitch, yaw):
        if(pitch != np.pi/2):
            return np.array([[1, sin(roll)*tan(pitch), cos(roll)*tan(yaw)],
                            [0, cos(roll), -sin(roll)],
                            [0, sin(roll)/cos(pitch), cos(roll)/cos(pitch)]])
        else:
            return np.array([[1, sin(roll) * tan(pitch), cos(roll) * tan(yaw)],
                             [0, cos(roll), -sin(roll)],
                             [0, sin(roll) / (cos(pitch) + 1e-100), cos(roll) / (cos(pitch) + 1e-100)]])

    ###########################
    # Callback function
    ###########################
    def NewInfoHover(self, hover_msg):

    	self.desired_att[0] = hover_msg.vx
        self.desired_att[1] = hover_msg.vy
        self.desired_ang_vel[2] = hover_msg.yawrate
        self.desired_thrust = hover_msg.zDistance
        rospy.loginfo("Received new hover info: " + str(hover_msg.vx) + ", " + str(hover_msg.vy) + ", " + str(hover_msg.yawrate) + ", " + str(hover_msg.zDistance))
    ###########################
    # Single step simulation
    ###########################
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

        self.cf_state.getMotorRotationSpeed()
        self.cf_state.addMotorsRotationsSpeed()
        self.cf_state.getForces()
        self.cf_state.getMomentums()

        new_state.lin_vel = np.array([0, 0, self.cf_state.forces[2]/self.cf_physical_params.DRONE_MASS]) \
                            - rotation_matrix * np.array([0, 0, self.cf_physical_params.G]) \
                            - np.cross(self.cf_state.ang_vel, self.cf_state.lin_vel)

        new_state.position = np.dot(np.transpose(rotation_matrix), self.cf_state.lin_vel)

        preoperation = self.cf_state.momentums - np.cross(self.cf_state.ang_vel,
                                            np.dot(self.cf_physical_params.INERTIA_MATRIX,
                                            self.cf_state.ang_vel))

        new_state.ang_vel = np.dot(self.cf_physical_params.INV_INERTIA_MATRIX, preoperation)
        
        new_state.attitude = np.dot(euler_matrix, self.cf_state.ang_vel)

        for i in range(0, 4):
            self.cf_state.position = self.cf_state.position + (new_state.position * self.cf_physical_params.DT_CF)
            self.cf_state.lin_vel = self.cf_state.lin_vel + (new_state.lin_vel * self.cf_physical_params.DT_CF)
            self.cf_state.attitude = self.cf_state.attitude + (new_state.attitude * self.cf_physical_params.DT_CF)
            self.cf_state.ang_vel = self.cf_state.ang_vel + (new_state.ang_vel * self.cf_physical_params.DT_CF)


    def run_att_pid(self):
        self.desired_ang_vel = np.array([self.roll_pid.update(self.desired_att[0], self.cf_state.attitude[0]),
                                        self.pitch_pid.update(self.desired_att[1], self.cf_state.attitude[1]),
                                        self.desired_ang_vel[2]])


    def run_ang_vel_pid(self):
        self.desired_rpy = np.array([self.wx_pid.update(self.desired_ang_vel[0], self.cf_state.ang_vel[0]),
                                     self.wy_pid.update(self.desired_ang_vel[1], self.cf_state.ang_vel[1]),
                                     self.wz_pid.update(self.desired_ang_vel[2], self.cf_state.ang_vel[2])])

        self.rpyt_2_motor_pwm()

    def rpyt_2_motor_pwm(self):

        # Inputs
        r = self.desired_rpy[0]
        p = self.desired_rpy[1]
        y = self.desired_rpy[2]
        thrust = self.desired_thrust

        ##########################
        # Function that transform the output
        # r, p, y, thrust of the PIDs,
        # into the values of the PWM
        # applied to each motor
        ##########################
        R = r / 2.0
        P = p / 2.0
        Y = y
        self.cf_state.motor_pwm[0] = self.cf_physical_params.PWM_MAX(thrust - R + P + Y)
        self.cf_state.motor_pwm[1] = self.cf_physical_params.PWM_MAX(thrust - R - P - Y)
        self.cf_state.motor_pwm[2] = self.cf_physical_params.PWM_MAX(thrust + R - P + Y)
        self.cf_state.motor_pwm[3] = self.cf_physical_params.PWM_MAX(thrust + R + P - Y)
        ### BASADO EN ESTA PARTE DEL FILMWARE:
        #motorPower.m1 = limitThrust(control->thrust - r + p + control->yaw);
        #motorPower.m2 = limitThrust(control->thrust - r - p - control->yaw);
        #motorPower.m3 =  limitThrust(control->thrust + r - p + control->yaw);
        #motorPower.m4 =  limitThrust(control->thrust + r + p - control->yaw);
        ### PERO TENGO QUE INVESTIGARLO
        
        for i in range(len(self.cf_state.motor_pwm)):
            self.cf_state.motor_rotation_speed[i] = 0.2685 * self.cf_state.motor_pwm[i] + 4070.3

    def publish_state(self):

        self.pub.publish(self.cf_state.position)


    def run(self):

        while(not rospy.is_shutdown()):

            if(self.att_pid_counter == self.att_pid_counter_max):
                self.att_pid_counter = 0
                self.run_att_pid()
                self.run_ang_vel_pid()
                rospy.loginfo("Counter pid IN IF: " + str(self.att_pid_counter))

            else:
                self.att_pid_counter = self.att_pid_counter + 1
                rospy.loginfo("Counter pid: " + str(self.att_pid_counter))

            if(self.out_pos_counter == self.out_pos_counter_max):
                self.out_pos_counter = 0
                self.publish_state()
                rospy.loginfo("Counter out IN IF: " + str(self.att_pid_counter))

            else:
                self.out_pos_counter = self.out_pos_counter + 1
                rospy.loginfo("Counter out: " + str(self.out_pos_counter))

            self.apply_simulation_step()
            rospy.loginfo("Simulation step done")

            # Wait for the cycle left time
            self.simulation_freq.sleep()
            rospy.loginfo("Sleep done")

if __name__ == '__main__':
    model = CF_model()
    model.run()
    rospy.spin()

    
