import numpy as np
import cv2

# capture images from camera #1
camera = cv2.VideoCapture(1)

# grab the current frame
(grabbed, frame) = camera.read()

cv2.imwrite('LGF1.png',frame)
camera.release()
cv2.destroyAllWindows()
