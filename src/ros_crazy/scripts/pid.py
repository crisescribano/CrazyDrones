class PID():

    def __init__(self, _kp, _ki, _kd, _max_i_value):

        # _kp, _ki ,_kd are the pid gains
        self.kp = _kp
        self.ki = _ki
        self.kd = _kd
        self.max_i_value = _max_i_value

    def update(self, desired, measurement):

        # The error between the desired state and
        # the measured one
        error = desired - measurement

        # The control output that we want to
        # estimate
        control_out = 0.0

        ###############################
        # ADD PID EQUATIONS HERE
        ###############################

        return control_out
