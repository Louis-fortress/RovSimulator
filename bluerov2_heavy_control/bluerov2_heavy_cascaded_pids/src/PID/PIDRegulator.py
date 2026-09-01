import numpy


class PIDRegulator:
    """1D PID regulator with filtered derivative action
    
    Continuous-time controller:

        u(s) = Kp*e(s) + Ki/s*e(s) + Kd*N*s/(s + N)*e(s)
    """
    def __init__(self, p, i, d, sat, n):
        self.p = p
        self.i = i
        self.d = d
        self.sat = sat
        self.n = n

        # PID states
        self.integral = 0.0
        self.prev_err = 0.0
        self.prev_t   = -1.0

        #filtered derivative state
        self.derivative = 0.0

    def __str__(self):
        msg = 'PID controller:'
        msg += '\n\tp=%f' % self.p
        msg += '\n\ti=%f' % self.i
        msg += '\n\td=%f' % self.d
        msg += '\n\tn=%f' % self.n
        msg += '\n\tsat=%f' % self.sat

        return msg

    def regulate(self, err, t):
        dt = t - self.prev_t
        if self.prev_t < 0.0:
            self.prev_err = err
            self.prev_t = t
            return 0.0

        #Proetection against invalid timestamps
        if dt <= 0.0:
            return(self.p*err + self.i*self.integral + self.d * self.derivative)

        self.integral += 0.5 * (err + self.prev_err)*dt

        #Discrete implementation:
        #
        # d_k = [N*(e_k-e_{k-1}) + d_{k-1}]
        #       --------------------------------
        #               1 + N*dt
        if self.n > 0:
            self.derivative = (self.n * (err - self.prev_err)
                               + self.derivative)/(1.0+self.n*dt)
        else:
            self.derivative = 0.0

        u = self.p*err + self.i*self.integral+self.d*self.derivative
        if abs(u) > self.sat:
            u = self.sat * numpy.sign(u)
            self.integral = 0.0

        self.prev_err = err
        self.prev_t = t
        return u
        