from numpy import interp
import math

class IKHelper():
  def __init__(self, tibia, femur):
    self.T = tibia
    self.F = femur

  def convert(self, g, a, b):
    servo_x = interp(g,[-80,100],[2100,500])
    #print "a = {}".format(a)
    servo_y = interp(a,[110,190],[1100,1700])
    #print "servo_y = {}".format(servo_y)
    beta_min = interp(servo_y,[1100,1700],[95,10])
    beta_max = interp(servo_y,[1100,1700],[170,85])
    #print beta_min, beta_max
    servo_z = interp(b,[beta_min,beta_max],[1600,1000])
    return (int(servo_x), int(servo_y), int(servo_z))

  def radsToDegs(self, x):
    return x*(180/math.pi)

  def getGamma(self, X, Y, Z):
    g = math.atan2(X,Y)
    return self.radsToDegs(g)

  def getAlpha(self, X, Y, Z):
    L1 = math.sqrt(math.pow(X,2) + math.pow(Y,2))
    L = math.sqrt(math.pow(Z,2) + math.pow(L1,2))
    if Z < 0:
      Z = abs(Z)
      a1 = math.acos(Z/L)
    else:
      a1 = math.asin(Z/L)+(math.pi/2)
    a2 = math.acos((math.pow(self.T,2)-math.pow(self.F,2)-math.pow(L,2))/(-2*self.F*L))
    a = a1+a2
    return self.radsToDegs(a)

  def getBeta(self, X, Y, Z):
    L1 = math.sqrt(math.pow(X,2) + math.pow(Y,2))
    L = math.sqrt(math.pow(Z,2) + math.pow(L1,2))
    b = math.acos((math.pow(L,2) - math.pow(self.T,2) - math.pow(self.F,2))/(-2*self.T*self.F))
    return self.radsToDegs(b)

  def getPWM(self, X, Y, Z):
    # Determine required joint angles
    g = self.getGamma(X, Y, Z)
    a = self.getAlpha(X, Y, Z)
    b = self.getBeta(X, Y, Z)
    # Determine PWM values
    retval = self.convert(g, a, b)
    return retval