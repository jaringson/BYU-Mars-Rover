
class PID:
    def __init__(self):
        self.ki = 0
        self.kp = 0
        self.kd = 0
        self.E = 0
        self.error_d1 = 0

    def SetGains(self, ki, kd, kp):
        self.ki = ki
        self.kd = kd
        self.kp = kp

    def GetOutputs(self,heading):

