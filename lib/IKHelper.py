from numpy import interp
import math

X_MAX_PWM = 525
X_MIN_PWM = 2000
Y_MAX_PWM = 1600
Y_MIN_PWM = 900
Z_MAX_PWM = 1400
Z_MIN_PWM = 800

X_MAX_ANGLE = 90
X_MIN_ANGLE = -90
Y_MAX_ANGLE = 185
Y_MIN_ANGLE = 100
Z_MAX_ANGLE = 55
Z_MIN_ANGLE = 25

I2C_PWM_MIN = 102
I2C_PWM_MAX = 510

US_PWM_MIN = 500
US_PWM_MAX = 2500

class IKHelper():
  def __init__(self, tibia, femur):
    self.T = tibia
    self.F = femur

  def convert(self, g, a, b):
    print g,a,b
    # Calculate joint angles for X and Y
    servo_x = interp(g,[X_MIN_ANGLE,X_MAX_ANGLE],[X_MIN_PWM,X_MAX_PWM])
    servo_y = interp(a,[Y_MIN_ANGLE,Y_MAX_ANGLE],[Y_MIN_PWM,Y_MAX_PWM])
    # Calculate current beta degrees due to mechanical structure
    a2 = a - 90
    #a3 = math.atan2(19,(15.5+18))
    b2 = 180 - a2
    #print "ANGLE: {0}, Tibia Length: {2}".format(b2,self.T)
    
    # Calculate how much more beta axis needs to rotate where delta_b is degrees
    # servo needs to rotate.
    delta_b = b2 - b
    # map servo_z rotation
    servo_z = interp(delta_b,[Z_MIN_ANGLE,Z_MAX_ANGLE],[Z_MIN_PWM,Z_MAX_PWM])
    #servo_r = interp(g,[-90,90],[2100,600])
    servo_r = 1350

    print "sending: {0},{1},{2},{3}\0\n".format(servo_x,servo_y,servo_z,servo_r)

    # Convert to 1/4096 scale for i2c board
    servo_x = interp(servo_x,[US_PWM_MIN,US_PWM_MAX],[I2C_PWM_MIN,I2C_PWM_MAX])
    servo_y = interp(servo_y,[US_PWM_MIN,US_PWM_MAX],[I2C_PWM_MIN,I2C_PWM_MAX])
    servo_z = interp(servo_z,[US_PWM_MIN,US_PWM_MAX],[I2C_PWM_MIN,I2C_PWM_MAX])
    servo_r = interp(servo_r,[US_PWM_MIN,US_PWM_MAX],[I2C_PWM_MIN,I2C_PWM_MAX])
    

    return (int(servo_x), int(servo_y), int(servo_z), int(servo_r))

  def radsToDegs(self, x):
    return x*(180/math.pi)

  def getGamma(self, X, Y, Z):
    g = math.atan2(X,Y)
    #leftside = (18*math.cos(g) - (math.pow(math.pow(X**2+Y**2,0.5)+18,2)) + math.pow(math.pow(X**2+Y**2,0.5),2))/(-2*(math.pow(X**2+Y**2,0.5)+18)*(math.pow(X**2+Y**2,0.5)))
    #print leftside
    #g2 = math.acos(leftside)
    #g = g - g2
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
