import cv2

import numpy as np

cap = cv2.VideoCapture(0)

vw = cv2.VideoWriter('out.avi',-1,20.0,(640,480))

while(cap.isOpened()):
    ret, frame = cap.read()
