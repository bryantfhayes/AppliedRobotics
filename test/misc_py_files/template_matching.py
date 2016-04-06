import cv2
import numpy as np
from matplotlib import pyplot as plt


# capture images from camera #1
camera = cv2.VideoCapture(1)

while True:
  # grab the current frame
  (grabbed, img_rgb) = camera.read()

  img_gray = cv2.cvtColor(img_rgb, cv2.COLOR_BGR2GRAY)
  template = cv2.imread('switch.png',0)
  w, h = template.shape[::-1]

  res = cv2.matchTemplate(img_gray,template,cv2.TM_CCOEFF_NORMED)
  threshold = 0.7
  loc = np.where( res >= threshold)
  for pt in zip(*loc[::-1]):
    cv2.rectangle(img_rgb, pt, (pt[0] + w, pt[1] + h), (0,0,255), 2)

  cv2.imshow("img_rgb", img_rgb)

  if cv2.waitKey(2) == 27:
    cv2.imwrite("pic.png", img_rgb);
    break
