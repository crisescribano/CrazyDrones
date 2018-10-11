class PID():

    def __init__(self, _kp, _ki, _kd, _max_i_value, _dt):

        # _kp, _ki ,_kd are the pid gains
        self.kp = _kp
        self.ki = _ki
        self.kd = _kd
        self.max_i_value = _max_i_value
		### Y max_pid_value??
        self.dt = _dt
		
		### error no hace falta tenerlo aqui, solo prev error
		### pero si lo quieres tener...usalo
        self.error = 0
        self.prev_error = 0

        # The control output that we want to
        # estimate
		### Lo mismo, no necesario, pero si lo tienes aqui, 
		### usalo
        self.output = 0

		### Lo mismo, no necesario, pero si lo tienes aqui, 
		### usalo
        self.control_out_P = 0
        self.control_out_D = 0
		
		### No los usas
        self.deriv = 0
        self.integ = 0

    def update(self, desired, measurement):

        # The error between the desired state and
        # the measured one
		### self.error? aunque no necesario
        error = desired - measurement

        ########################
        # PID EQUATIONS
        ########################

        #P:
		### self.output? aunque no necesario
        output = 0
        control_out_P = self.kp * error
        output += control_out_P

        #D:
		### dt y prev_error necesitan self. 
		### Sin el self, se entienden como nuevas
		### variables creadas cada vez que llamas update
		### Y eso funciona para la variable error por ejemplo
		### por que en cada update tenemos un nuevo error. O 
		### en output. Pero en las variables cuyo valor dependa
		### de iteraciones anteriores o otras funciones, necesitas 
		### el self, porque asi el valor es parte de la clase
		### y se mantiene:) 
		### Entonces necesitas tener cuidad con los self.
		### En este caso, self.dt, self.prev_error
        deriv = (error - prev_error) / dt
		### No neceitas control_out_D, puedes hacer 
		### output += self.kd*deriv
        control_out_D = kd * deriv 
        output += control_out_D

        ### Aqui lo mismo. Integ tiene que ser self.
		### ya que su valor depende de iteraciones anteriores
        integ += error * dt 
		### Self!! Aqui lo necesitas por que es una variable 
		### de la clase, inicializada en el init, y necesitas poder
		### coger el valor que le diste. Sin el self., es una variable
		### no inizializada y que solo afecta a esta iteracion
        if (max_i_value != 0) 
			### Aparte de los self...que es constrain? No se si
			### numpy tiene una funcion, si no haz: 
			### max(min(integ, max_value_i), -max_i_value) 
			### con self claro
            integ = constrain(integ, -max_i_value, max_i_value);
		### Los self y puedes hacer directamente output += self.ki*self.integ
        control_out_I = ki * integ
        output += control_out_I

		### Self
        if (max_pid_value != 0)
			### constraint?
            output = constraint(output, -max_pid_value, max_pid_value)
		### SELF!
        prev_error = error
        
        return output
