import numpy as np
import cv2
import time
from collections import deque
import argparse

# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video",
      help="path to the (optional) video file")
ap.add_argument("-b", "--buffer", type=int, default=64,
      help="max buffer size")
args = vars(ap.parse_args())

# define the lower and upper boundaries of the "green"
# ball in the HSV color space, then initialize the
# list of tracked points

#151,38,208   215,231,255

greenLower = (151, 38, 208)
greenUpper = (215, 231, 255)
pts = deque(maxlen=args["buffer"])

#Captures the right camera of the Minoru
#Sets camera resolution to be 320x240 for testing purposes
right = cv2.VideoCapture(1) 
#right.set(3,320)
#right.set(4,240)

#Captures the left camera of the Minoru
#Sets camera resolution to be 320x240 for testing purposes
left = cv2.VideoCapture(2)
#left.set(3, 320)
#left.set(4, 240)

#Sets predefined values for camera calibration. 
window_size = 4
min_disp = 16
num_disp = 64-min_disp

timeOutLen = 180
numFrames = 0
timeStart = time.time()
RRImg = 0
RLImg = 0
CvtRect = 0
DispImg = 0
DispComp = 0
CntFnd = 0

#Infinite loop for video capture/ processing 
while True:
  # grab the current frame
  (grabbed, frame) = left.read()
 
  # resize the frame, blur it, and convert it to the HSV
  # color space
  # blurred = cv2.GaussianBlur(frame, (11, 11), 0)
  hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
  
  # construct a mask for the color "green", then perform
  # a series of dilations and erosions to remove any small
  # blobs left in the mask
  mask = cv2.inRange(hsv, greenLower, greenUpper)
  #mask = cv2.erode(mask, None, iterations=2)
  #mask = cv2.dilate(mask, None, iterations=2)
 
  cv2.imshow("Mask", mask)

  numFrames += 1

  if cv2.waitKey(2) == 27:
    timeEnd = time.time()
    break

print "Frames per second:", (numFrames/(timeEnd - timeStart))

right.release()
left.release()
cv2.destroyAllWindows()
