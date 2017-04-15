from math import *

class ArmConversion:
    def __init__(self):
    
        self.las_min = 13
        self.las_max = 16
        self.lae_min = 15
        self.lae_max = 20
        
        self.sx = 2.5
        self.sy = 3
        self.A = 15.375
        self.la = 20
        self.eh = 3.0+1/16
        self.d1 = 0.5
        self.d2 = 0.75
        self.d3 = 2.25
        self.d4 = 2
        
        self.sh = sqrt(self.sx**2 + self.sy**2)
        self.D = sqrt(self.d1**2 + (self.la-self.d4)**2)
        self.gamma1 = asin(self.d1/self.D)
        self.E = sqrt(self.d3**2 + self.eh**2)
        self.gamma6 = acos(self.eh/self.E)
        self.B = sqrt(self.A**2+self.d2**2)
        self.beta2 = acos(self.A/self.B)
        self.alpha1 = acos(self.sx/self.sh)
        
    def getLengths(self,theta2,theta3):

        alpha4 = pi - self.alpha1 - theta2
        beta1 = alpha4 - self.beta2
        gamma3 = theta3 - self.gamma1 - self.gamma6
        
        lae = (self.D**2 - 2*cos(gamma3)*self.D*self.E + self.E**2)**(1/2)
        las = self.sh*cos(beta1) + (self.sh**2*cos(beta1)**2 + self.B**2 - self.sh**2)**(1/2)
        
        las_percent = -(las-self.las_min)/(self.las_min-self.las_max)
        lae_percent = -(lae-self.lae_min)/(self.lae_min-self.lae_max)
        print lae, las
	return [las_percent,lae_percent]
        
    def getAngles(self,las_percent,lae_percent):
    
        las = self.las_min + las_percent*(self.las_max-self.las_min)
        lae = self.lae_min + lae_percent*(self.lae_max-self.lae_min)
        
        beta1 = acos((las**2+self.sh**2-self.B**2)/(2*self.D*self.E))
        gamma3 = acos((self.D**2+self.E**2-lae**2)/(2*self.D*self.E))
        alpha4 = beta1+self.beta2
        
        theta2 = pi-self.alpha1-alpha4
        
        theta3 = self.gamma1 + gamma3 + self.gamma6
        return [theta2, theta3]
        
if __name__=="__main__":
	a = ArmConversion()
	print a.getLengths(0,0)
