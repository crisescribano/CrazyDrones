class PID():

    def __init__(self, _kp, _ki, _kd, _max_i_value, _dt):

        # _kp, _ki ,_kd are the pid gains
        self.kp = _kp
        self.ki = _ki
        self.kd = _kd
        self.max_i_value = _max_i_value
        self.dt = _dt

        self.error = 0
        self.prev_error = 0

        # The control output that we want to
        # estimate
        self.output = 0

        self.control_out_P = 0
        self.control_out_D = 0

        self.deriv = 0
        self.integ = 0

    def update(self, desired, measurement):

        # The error between the desired state and
        # the measured one
        error = desired - measurement

        ########################
        # PID EQUATIONS
        ########################

        #P:
        output = 0
        control_out_P = self.kp * error
        output += control_out_P

        #D:
        deriv = (error - prev_error) / dt
        control_out_D = kd * deriv 
        output += control_out_D

        #I
        integ += error * dt 
        if (max_i_value != 0) 
            integ = constrain(integ, -max_i_value, max_i_value);
        control_out_I = ki * integ
        output += control_out_I

        if (max_pid_value != 0)
            output = constraint(output, -max_pid_value, max_pid_value)

        prev_error = error
        
        return output
