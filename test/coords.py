import math, serial, time
from numpy import interp

T = 15.5
F = 14

def convert(g, a, b):
  servo_x = interp(g,[-80,100],[2100,500])
  #print "a = {}".format(a)
  servo_y = interp(a,[110,190],[1100,1700])
  #print "servo_y = {}".format(servo_y)
  beta_min = interp(servo_y,[1100,1700],[95,10])
  beta_max = interp(servo_y,[1100,1700],[170,85])
  #print beta_min, beta_max
  servo_z = interp(b,[beta_min,beta_max],[1600,1000])
  return (int(servo_x), int(servo_y), int(servo_z))

def radsToDegs(x):
  return x*(180/math.pi)

def getGamma(X, Y, Z):
  gamma = math.atan2(X,Y)
  return radsToDegs(gamma)

def getAlpha(X, Y, Z):
  L1 = math.sqrt(math.pow(X,2) + math.pow(Y,2))
  L = math.sqrt(math.pow(Z,2) + math.pow(L1,2))
  if Z < 0:
    Z = abs(Z)
    a1 = math.acos(Z/L)
  else:
    a1 = math.asin(Z/L)+(math.pi/2)
  a2 = math.acos((math.pow(T,2)-math.pow(F,2)-math.pow(L,2))/(-2*F*L))
  a = a1+a2
  return radsToDegs(a)

def getBeta(X, Y, Z):
  L1 = math.sqrt(math.pow(X,2) + math.pow(Y,2))
  L = math.sqrt(math.pow(Z,2) + math.pow(L1,2))
  b = math.acos((math.pow(L,2) - math.pow(T,2) - math.pow(F,2))/(-2*T*F))
  return radsToDegs(b)

def cartesianToDegs(X, Y, Z):
  g = getGamma(X, Y, Z)
  a = getAlpha(X, Y, Z)
  b = getBeta(X, Y, Z)
  return (g, a, b)


def test():
  i = 0
  for y in xrange(10,27,4):
    for x in [-5,5]:
      print x,y
      g,a,b = cartesianToDegs(x, y, -7+i)
      sx,sy,sz = convert(g,a,b)
      msg = "{0},{1},{2},{3}\n".format(sx,sy,sz,1490)
      ser.write(msg)
      time.sleep(2)
    if y == 10:
      i += 1

port = '/dev/cu.usbmodem1411'
ser = serial.Serial(port,115200,timeout=5)


test()
time.sleep(5000)

while True:
  print "Enter X,Y,Z:"
  coords = raw_input()
  coords = coords.split(',')
  coords = map(int, coords)
  print "G, A, B:"
  print cartesianToDegs(coords[0], coords[1], coords[2])
  print "PWM:"
  g,a,b = cartesianToDegs(coords[0], coords[1], coords[2])
  print convert(g,a,b)
  sx,sy,sz = convert(g,a,b)
  msg = "{0},{1},{2},{3}\n".format(sx,sy,sz,1490)
  ser.write(msg)
  print "sent: {}".format(msg)

