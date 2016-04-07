import IKHelper,serial,socket,math

UDP_IP   = "edison.local"
UDP_PORT = 21234 

class Tool(object):
  def __init__(self,x,y,z):
    self.x, self.y, self.z = (x,y,z)

# Robotic arm class responsible for sending movement PWM signals to 
# an Arduino or some microcontroller device which then writes those 
# values to individual servos. 
class SerialArm(object):
  def __init__(self, tibia, femur, tool, port=None, z_offset=0, wireless=False):
    self.x, self.y, self.z, self.r = (1350,1500,1300,1350)
    self.port = port
    self.tool = tool
    self.wireless = wireless
    self.z_offset = z_offset
    if self.port:
      self._ser = serial.Serial(port,115200,timeout=5)
    elif self.wireless:
      self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    self.ik = IKHelper.IKHelper(math.pow(((tibia+tool.y)**2)+(tool.z**2), 0.5), femur)

  def send(self, msg):
    if self.port:
      self._ser.write(msg)
    elif self.wireless:
      self.sock.sendto(msg, (UDP_IP, UDP_PORT))

  def updateArm(self):
  	self.send("{0},{1},{2},{3}\0\n".format(self.x,self.y,self.z,self.r))

  def getPosition(self):
  	return (self.x, self.y, self.z, self.r)

  def _directWrite(self,x,y,z,r):
      self.send("{0},{1},{2},{3}\0\n".format(x,y,z,r))

  # Takes either a tuple (X, Y, Z) or individual X, Y, Z args
  def setPosition(self, X, Y=None, Z=None):
    if isinstance(X, tuple) or isinstance(X, list):
      #self.x, self.y, self.z, self.r = self.ik.getPWM(X[0] - self.tool.x, X[1] - self.tool.y, X[2] + self.tool.z - self.z_offset)
      self.x, self.y, self.z, self.r = self.ik.getPWM(X[0], X[1], X[2] - self.z_offset)
    else:
      #self.x, self.y, self.z, self.r = self.ik.getPWM(X - self.tool.x, Y - self.tool.y, Z + self.tool.z - self.z_offset)
      self.x, self.y, self.z, self.r = self.ik.getPWM(X, Y, Z - self.z_offset)
    self.updateArm()

