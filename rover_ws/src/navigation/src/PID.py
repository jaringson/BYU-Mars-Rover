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

    def reset(self):
        self.X = 0
        self.error_d1 = 0
        self.x_d1 = 0
        self.xdot = 0

    def execute(self, inputs):

        x_c = inputs[0]
        x = inputs[1]
        t = inputs[2]

        # Compute the dirty derivative
        self.xdot = (2*self.sigma-self.dt)/(2*self.sigma+self.dt)*self.xdot +\
                    2/(2*self.sigma+self.dt)*(x-self.x_d1)
        self.x_d1 = x

        # Compute the integrator
        error = self.keepangle(x_c - x)
        self.X += self.keepangle(error*self.dt)
        self.error_d1 = error

        # Compute the output
        dE = (error - self.error_d1) / self.dt
        u_unsat = self.kp*error + self.ki*self.X * self.kd*dE
        u = self.sat(u_unsat)

        # Anti-windup
        if self.ki != 0:
            self.X += self.dt/self.ki*(u-u_unsat)

        print error
        return u

    def sat(self, val):
        if val > self.lim:
            return self.lim
        elif val < -self.lim:
            return -self.lim
        else:
            return val

    def keepangle(self,x):
        return (x-math.pi) % (2*math.pi) - math.pi



