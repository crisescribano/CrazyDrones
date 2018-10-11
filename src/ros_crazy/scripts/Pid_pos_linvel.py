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
        self.pub = rospy.Publisher("cmd_hover", Hover, queue_size=1)

		### Esta esta bien, quizas haya que añadir un paramtero desde el launch para que sepamos para que drone
		### es : y seria "/" + topic + "cmd_position" o algo asi
        rospy.Subscriber("cmd_position", Position, NewPointToAchieve)

        # System state: position, linear velocities,
        # attitude and angular velocities


        # Import the crazyflie physical paramters
        #     - These parameters are obtained from different sources.
        #     - For these parameters and the dynamical equations refer
        #       to : DESIGN OF A TRAJECTORY TRACKING CONTROLLER FOR A
        #            NANOQUADCOPTER
        #            Luis, C., & Le Ny, J. (August, 2016)
		
        self.cf_physical_params = CF_parameters()

        # Import the PID gains (from the firmware)
        self.cf_pid_gains = CF_pid_params()


        ### DUDA: ESTA BIEN ESTA FRECUENCIA?
        self.simulation_freq = rospy.Rate(int(1/self.cf_physical_params.DT_CF))


        ######################
        # Initialize PID
        ######################
		
        self.desired_lin_vel = np.zeros(3)      # Obtained from 'pos controller'
        self.desired_pitch_roll = np.zeros(2)   # Obtained from 'lin vel controller'
        self.desired_yawrate = 0                # Obtained from 'yaw controller'
        self.desired_thrust = 0                 # Obtained from cmd_hover


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
						  
		    ######################
        # Yaw
        ######################

        self.yaw_pid = PID(self.cf_pid_gains.KP_YAW,
                          self.cf_pid_gains.KI_YAW,
                          self.cf_pid_gains.KD_YAW,
                          self.cf_pid_gains.INT_MAX_YAW,
                          self.cf_pid_gains.YAW_DT)


        def NewPointToAchieve(position):
            self.


        def run_pos_pid(self):
        	self.desired_lin_vel = np.array([self.x_pid.update(self.desired_lin_vel[0], self.cf_state.lin_vel[0]),
                                           self.y_pid.update(self.desired_lin_vel[1], self.cf_state.lin_vel[1]),
                                           self.z_pid.update(self.desired_lin_vel[2], self.cf_state.lin_vel[2])])
 
        def run_lin_vel_pid(self):
        	self.desired_pitch_roll = np.array([self.x_pid.update(self.desired_lin_vel[0], self.cf_state.lin_vel[0]),
                                       self.y_pid.update(self.desired_lin_vel[1], self.cf_state.lin_vel[1]),
                                       self.z_pid.update(self.desired_lin_vel[2], self.cf_state.lin_vel[2])])

        def publish_state(self):
		  ### DUDA: NO SE SI PUEDO PONER MSG = Hover() DIRECTAMENTE Y 
      ### LO RECONOCE COMO EL TIPO DE MENSAJE

      ### DUDA: LO QUE PIDE EL Hover ES DIRECTAMENTE zDistance ENTONCES
      ### ENTIENDO QUE ES EL THRUST DIRECTAMENTE SIN EL PID PERO NOT SURE

          msg = Hover()
          msg.vx = desired_pitch_roll[0] 
          msg.vx = desired_pitch_roll[1] 
          msg.yawrate = desired_yawrate
          msg.zDistance = desired_thrust * 1000

          self.pub.publish(msg)

        def run(self):

        	while(not rospy.is_shutdown()):
				### No has declarado ni inicializado estos counters,
				### El del yaw tiene que ir a 500 Hz, asique tiene que 
				### runearse en otro 
            	if(self.pos_pid_counter == self.pos_pid_counter_max):
                	self.pos_pid_counter = 0
                	self.run_pos_pid()
                	self.run_lin_vel_pid()
            	else:
                	self.pos_pid_counter = self.pos_pid_counter + 1

              if(self.yaw_pid_counter == self.yaw_pid_counter_max):
                  self.yaw_pid_counter = 0
                  self.run_yaw_pid()
              else:
                  self.yaw_pid_counter = self.yaw_pid_counter + 1

              if(self.out_pos_counter == self.out_pos_counter_max):
                self.out_pos_counter = 0
                self.publish_state()
              else:
                self.out_pos_counter = self.out_pos_counter + 1
            	rospy.spin()

            	# Wait for the cycle left time
				### Esto no lo has inicializado tampoco 
            	self.simulation_freq.sleep()


if __name__ == '__main__':
    pid_pos = PID_pos_linvel()
    pid_pos.run()
