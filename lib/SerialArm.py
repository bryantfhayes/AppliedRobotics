import IKHelper,serial

# Robotic arm class responsible for sending movement PWM signals to 
# an Arduino or some microcontroller device which then writes those 
# values to individual servos. 
class SerialArm(object):
  def __init__(self, tibia, femur, port=None):
    self.x, self.y, self.z, self.r = (1426,1500,1500,1490)
    self.port = port
    if self.port:
    	self._ser = serial.Serial(port,115200,timeout=5) 
    self.ik = IKHelper.IKHelper(tibia, femur)

  def updateArm(self):
  	if self.port:
  		self._ser.write("{0},{1},{2},{3}\n".format(self.x,self.y,self.z,self.r))

  def getPosition(self):
  	return (self.x, self.y, self.z, self.r)

  def _directWrite(self,x,y,z,r):
    if self.port:
      self._ser.write("{0},{1},{2},{3}\n".format(x,y,z,r))
    else:
      print "Serial port required"

  # Takes either a tuple (X, Y, Z) or individual X, Y, Z args
  def setPosition(self, X, Y=None, Z=None):
    if isinstance(X, tuple) or isinstance(X, list):
      self.x, self.y, self.z = self.ik.getPWM(X[0], X[1], X[2])
    else:
      self.x, self.y, self.z = self.ik.getPWM(X,Y,Z)
    self.updateArm()
