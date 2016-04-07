import serial, sys
# Add reference to 'lib' folder
sys.path.append('../lib')
import SerialArm

if len(sys.argv) < 2:
  print "No serial port given"
  exit()

print "Initializing arm at: {}".format(sys.argv[1])
arm = SerialArm.SerialArm(14, 15.5, SerialArm.Tool(0,0,0), port=sys.argv[1])

while True:
  print "INPUT:"
  cmd = raw_input()
  cmd = cmd.split(',')
  cmd = map(int, cmd)
  arm._directWrite(cmd[0],cmd[1],cmd[2],cmd[3])
  print "Wrote: {}".format(cmd)
