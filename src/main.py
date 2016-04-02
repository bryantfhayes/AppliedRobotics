import serial, syslog, time
import Tkinter as tk

# servo_1 = rod rotation (r)  700-2400 PIN(2)
# servo_2 = up/down (z)      1000-1600 PIN(3)
# servo_3 = forward/back (y) 1100-1700 PIN(4)
# servo_4 = main rotation (x) 500-2100 PIN(5)
ranges = {"X" : [500, 2100], "Y" : [1100, 1700], "Z" : [1000, 1600], "R" : [700, 2400]}
SERVO_DEFAULT = {"X" : 1460, "Y" : 1700, "Z" : 1000, "R" : 1240}
HOME_STATE = (1460,1700,1000,1240)
MOVE_SPEED = 20
DEBUG = False

class Servo(object):
  def __init__(self, name):
    self._minRange = ranges[name][0]
    self._maxRange = ranges[name][1]
    self._value = SERVO_DEFAULT[name]
  
  def setValue(self, val):
    try:
      val = int(val)
      if val > self._minRange and val < self._maxRange:
        self._value = val
      elif val <= self._minRange:
        self._value = self._minRange
        if DEBUG:
          print "[MIN]Value out of bounds! Setting to: {}".format(self._minRange)
      else:
        self._value = self._maxRange
        if DEBUG:
          print "[MAX]Value out of bounds! Setting to: {}".format(self._maxRange)
    except:
      if DEBUG:
        print "Invalid value!"

  def getValue(self):
    return self._value

  def add(self, chg):
    try:
      self.setValue(self._value + int(chg))
    except:
      if DEBUG:
        print "Invalid value!"

class Arm(object):
  def __init__(self, port):
    self.x = Servo("X")
    self.y = Servo("Y")
    self.z = Servo("Z")
    self.r = Servo("R")
    self._ser = serial.Serial(port,115200,timeout=5) 
    self._states = []
    time.sleep(1)
    self.update()

  def update(self):
    self._ser.write("{0},{1},{2},{3}\n".format(self.x.getValue(),self.y.getValue(),self.z.getValue(),self.r.getValue()))

  def getState(self):
    return (self.x.getValue(), self.y.getValue(), self.z.getValue(), self.r.getValue())
  
  def setState(self, state):
    self.x.setValue(state[0])
    self.y.setValue(state[1])
    self.z.setValue(state[2])
    self.r.setValue(state[3])
    self.update()

  def record(self, state):
    self._states.append(state)

  def clear(self):
    self._states = []

  def playback(self):
    for state in self._states:
      self.setState(state)
      time.sleep(3)

  def home(self):
    self.setState(HOME_STATE);

  def back(self):
    if len(self._states) > 0:
      self.setState(self._states[-1])

def onKeyPress(event):
  global arm
  if event.char == "w":
    arm.z.add(MOVE_SPEED)
  elif event.char == "s":
    arm.z.add(-MOVE_SPEED)
  
  if event.char == "a":
    arm.x.add(MOVE_SPEED)
  elif event.char == "d":
    arm.x.add(-MOVE_SPEED)
  
  if event.char == "q":
    arm.y.add(MOVE_SPEED)
  elif event.char == "e":
    arm.y.add(-MOVE_SPEED)
  
  if event.char == "z":
    arm.r.add(MOVE_SPEED)
  elif event.char == "x":
    arm.r.add(-MOVE_SPEED)

  elif event.char == "r":
    state = arm.getState()
    arm.record(state)
    print "Recorded: {}".format(state)
  elif event.char == "c":
    arm.clear()
    print "Memory Cleared"
  elif event.char == "p":
    print "Playing back from memory..."
    arm.playback()
    print "Playback Complete."
  elif event.char == "b":
    arm.back()

  elif event.char == "h":
    arm.home()

  else:
    if DEBUG:
      print "pressed something else"
  
  if event.char in ["w","a","s","d","q","e","z","x"]:
    arm.update()

arm = Arm('/dev/cu.usbmodem1411')

root = tk.Tk()
root.geometry('300x200')
text = tk.Text(root, background='black', foreground='white', font=('Comic Sans MS', 12))
text.pack()
root.bind('<KeyPress>', onKeyPress)
root.mainloop()
