import rospy
import numpy as np
from math import cos, sin, tan

from CrazyFlie_dynamical_model import CF_state
from cf_physical_parameters import CF_parameters
from cf_pid_params import CF_pid_params
from pid import PID  
3

class PID_pos_linvel():

    def __init__(self):

        rospy.init_node("pid_pos_linvel", anonymous=True)
        self.pub = rospy.Publisher("pitch_roll_topic", GenericLogData, queue_size=1)
        rospy.Subscriber("external_position", Position, ??????????)
        rospy.Subscriber("cmd_position", Position, ??????????)

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

        self.desired_lin_vel = np.zeros(3)
        self.desired_att = np.zeros(3)


        ######################
        # Position
        ######################

        self.x_pid = PID(self.cf_pid_gains.KP_X,
                          self.cf_pid_gains.KI_X,
                          self.cf_pid_gains.KD_X,
                          self.cf_pid_gains.INT_MAX_X,
                          self.cf_pid_gains.X_DT)

        self.y_pid = PID(self.cf_pid_gains.KP_Y,
                          self.cf_pid_gains.KI_Y,
                          self.cf_pid_gains.KD_Y,
                          self.cf_pid_gains.INT_MAX_Y,
                          self.cf_pid_gains.Y_DT)

        self.z_pid = PID(self.cf_pid_gains.KP_Z,
                          self.cf_pid_gains.KI_Z,
                          self.cf_pid_gains.KD_Z,
                          self.cf_pid_gains.INT_MAX_Z,
                          self.cf_pid_gains.Z_DT)


        ######################
        # Linear velocities
        ######################


        self.vx_pid = PID(self.cf_pid_gains.KP_VX,
                          self.cf_pid_gains.KI_VX,
                          self.cf_pid_gains.KD_VX,
                          self.cf_pid_gains.INT_MAX_VX,
                          self.cf_pid_gains.VX_DT)

        self.vy_pid = PID(self.cf_pid_gains.KP_VY,
                          self.cf_pid_gains.KI_VY,
                          self.cf_pid_gains.KD_VY,
                          self.cf_pid_gains.INT_MAX_VY,
                          self.cf_pid_gains.VY_DT)

        self.vz_pid = PID(self.cf_pid_gains.KP_VZ,
                          self.cf_pid_gains.KI_VZ,
                          self.cf_pid_gains.KD_VZ,
                          self.cf_pid_gains.INT_MAX_VZ,
                          self.cf_pid_gains.VZ_DT)


        def run_pos_pid(self):
        	self.desired_lin_vel = np.array([self.x_pid.update(self.desired_lin_vel[0], self.cf_state.lin_vel[0]),
                                           self.y_pid.update(self.desired_lin_vel[1], self.cf_state.lin_vel[1]),
                                           self.z_pid.update(self.desired_lin_vel[2], self.cf_state.lin_vel[2])])
 
        def run_lin_vel_pid(self):
        	self.desired_att = np.array([self.x_pid.update(self.desired_lin_vel[0], self.cf_state.lin_vel[0]),
                                       self.y_pid.update(self.desired_lin_vel[1], self.cf_state.lin_vel[1]),
                                       self.z_pid.update(self.desired_lin_vel[2], self.cf_state.lin_vel[2])])

        def publish_state(self):
          for i in CF_parameters().NUM_MOTORS:
            self.pub.publish(ATTITUDE)


        def run(self):

        	while(not rospy.is_shutdown()):


            	if(self.pos_pid_counter == self.pos_pid_counter_max):
                	self.pos_pid_counter = 0
                	self.run_pos_pid()
                	self.run_lin_vel_pid()
            	else:
                	self.pos_pid_counter = self.pos_pid_counter + 1

              if(self.out_pos_counter == self.out_pos_counter_max):
                self.out_pos_counter = 0
                self.publish_state()
              else:
                self.out_pos_counter = self.out_pos_counter + 1
            	rospy.spin()

            	# Wait for the cycle left time
            	self.simulation_freq.sleep()


if __name__ == '__main__':
    pid_pos = PID_pos_linvel()
    pid_pos.run()
