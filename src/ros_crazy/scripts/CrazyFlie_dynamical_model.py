#!/usr/bin/env python

import rospy
import numpy as np
from cf_physical_parameters import CF_parameters

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
        self.simulation_freq = rospy.Rate(1000)

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

    ###########################
    # Math operations functions
    ###########################

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
    def run(self):

        while(not rospy.is_shutdown()):

            # Apply the simulation step

            # Wait for the cycle left time
            self.simulation_freq.sleep()
