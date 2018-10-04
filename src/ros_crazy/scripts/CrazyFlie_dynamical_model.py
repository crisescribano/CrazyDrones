#!/usr/bin/env python

import rospy
import numpy as np
from cf_physical_parameters import CF_parameters
from cf_pid_params import CF_pid_params
from pid import PID

class CF_state():

    def __init__(self):
        self.position = np.zeros(3)
        self.lin_vel = np.zeros(3)
        self.attitude = np.zeros(3)
        self.ang_vel = np.zeros(3)
        self.motor_pwm = np.zeros(CF_parameters().NUM_MOTORS)

class CF_model():

    def __init__(self):

        # Main CF variables initialization (if needed)
        self.simulation_freq = rospy.Rate(int(1/self.cf_physical_params.DT_CF))

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

        ######################
        # Initialize PID
        ######################

        # Out from the PIDs, values of
        # r, p, y, thrust
        self.desired_rpy = np.zeros(3)

        # Comes from the external position controller
        self.desired_thrust = 0.0

        #Comes from the external yaw position controller
        self.desired_yaw_rate = 0.0

        ######################
        # Angular velocities
        ######################

        self.wx_pid = PID(self.cf_pid_gains.KP_WX,
                          self.cf_pid_gains.KI_WX,
                          self.cf_pid_gains.KD_WX,
                          self.cf_pid_gains.INT_MAX_WX)

        self.wy_pid = PID(self.cf_pid_gains.KP_WY,
                          self.cf_pid_gains.KI_WY,
                          self.cf_pid_gains.KD_WY,
                          self.cf_pid_gains.INT_MAX_WY)

        self.wz_pid = PID(self.cf_pid_gains.KP_WZ,
                          self.cf_pid_gains.KI_WZ,
                          self.cf_pid_gains.KD_WZ,
                          self.cf_pid_gains.INT_MAX_WZ)

        self.desired_ang_vel = np.zeros(3)

        ######################
        # Attitudes
        ######################

        self.roll_pid = PID(self.cf_pid_gains.KP_ROLL,
                          self.cf_pid_gains.KI_ROLL,
                          self.cf_pid_gains.KD_ROLL,
                          self.cf_pid_gains.INT_MAX_ROLL)

        self.pitch_pid = PID(self.cf_pid_gains.KP_PITCH,
                          self.cf_pid_gains.KI_PITCH,
                          self.cf_pid_gains.KD_PITCH,
                          self.cf_pid_gains.INT_MAX_PITCH)

        self.att_pid_counter = 0
        self.att_vel_pid_counter_max = int( self.cf_physical_params.DT_ATT_PIDS / self.cf_physical_params.DT_CF) - 1

        self.desired_att = np.zeros(3)

        ############################
        # Communication control
        ############################
        self.counter_out_pos = 0
        self.counter_out_pos_max = int(self.cf_physical_params.DT_COMMUNICATION / self.cf_physical_params.DT_CF) - 1


    ###########################
    # Math operations functions
    ###########################

    # Add math operations here if any:)

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

        ###########################
        # ADD SYSTEM EQUATIONS HERE
        ###########################

        # Update the state of the system
        self.cf_state = new_state

    def rpyt_2_motor_pwm(self):

        # Inputs
        r = self.desired_rpy[0]
        p = self.desired_rpy[1]
        y = self.desired_rpy[2]
        thrust = self.desired_thrust

        # Outputs: new values of self.cf_state.motor_pwm

        ##########################
        # Function that transform the output
        # r, p, y, thrust of the PIDs,
        # into the values of the PWM
        # applied to each motor
        ##########################

    def run_ang_vel_pid(self):
        self.desired_rpy = np.array([self.wx_pid.update(self.desired_ang_vel[0], self.cf_state.ang_vel[0]),
                                     self.wy_pid.update(self.desired_ang_vel[1], self.cf_state.ang_vel[1]),
                                     self.wz_pid.update(self.desired_ang_vel[2], self.cf_state.ang_vel[2])])

        self.rpyt_2_motor_pwm()

    def run_att_pid(self):
        self.desired_att = np.array([self.roll_pid.update(self.desired_att[0], self.cf_state.attitude[0]),
                                     self.pitch_pid.update(self.desired_att[1], self.cf_state.attitude[1]),
                                     self.roll_pid.update(self.desired_att[2], self.cf_state.attitude[2])])

    def publish_state(self):
        pass



    def run(self):

        while(not rospy.is_shutdown()):

            self.apply_simulation_step()

            if(self.att_pid_counter == self.att_pid_counter_max):
                self.att_pid_counter = 0
                self.run_ang_vel_pid()
                self.run_att_pid()
            else:
                self.att_pid_counter_max = self.att_pid_counter_max + 1

            if(self.counter_out_pos == self.counter_out_pos_max):
                self.counter_out_pos = 0
                self.publish_state()

            rospy.spin()
            # Wait for the cycle left time
            self.simulation_freq.sleep()


