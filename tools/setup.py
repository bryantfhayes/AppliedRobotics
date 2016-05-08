import serial

ser = serial.Serial('/dev/ttyMFD1', 9600, timeout=3)
a = ser.read(100)
print a

