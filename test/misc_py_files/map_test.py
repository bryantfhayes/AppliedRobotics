from numpy import interp

def convert(g, a, b):
  servo_x = interp(g,[-80,110],[2100,500])
  servo_y = interp(a,[110,190],[1100,1700])
  servo_z = interp(b,[20,75],[1600,1000])
  return (servo_x, servo_y, servo_z)

while True:
  print "Enter X,Y,Z"

  print convert(g, a, b)
