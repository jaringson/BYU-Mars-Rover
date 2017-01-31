import math


class PID:
    def __init__(self, params):
        self.ki = params.ki
        self.kp = params.kp
        self.kd = params.kd
        self.dt = params.dt
        self.sigma = params.sigma

        self.X = 0
        self.error_d1 = 0
        self.x_d1 = 0
        self.xdot = 0

        self.lim = 1e5

    def set_gains(self, ki, kd, kp):
        self.ki = ki
        self.kd = kd
        self.kp = kp

    def execute(self, inputs):

        x_c = inputs[0]
        x = inputs[1]
        t = inputs[2]

        # Compute the dirty derivative
        self.xdot = (2*self.sigma-self.dt)/(2*self.sigma+self.dt)*self.xdot +\
                    2/(2*self.sigma+self.dt)*(x-self.x_d1)
        self.x_d1 = x

        # Compute the integrator
        error = x_c - x
        self.X += self.dt/2*(error + self.error_d1)
        self.error_d1 = error

        # Compute the output
        u_unsat = self.kp*error + self.ki*self.X - self.kd*self.xdot
        u = self.sat(u_unsat)

        # Anti-windup
        if self.ki != 0:
            self.X += self.dt/self.ki*(u-u_unsat)

        return u

    def sat(self, val):
        if val > self.lim:
            return self.lim
        elif val < -self.lim:
            return -self.lim
        else:
            return val


