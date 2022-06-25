import cv2
import numpy as np

from datetime import datetime

# datetime object containing current date and time
now = datetime.now()
dt_string = now.strftime("%d-%m-%Y_%H_%M_%S")
print("date and time =", dt_string)

cap = cv2.VideoCapture(2)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 30)
# cap = cv2.VideoCapture('outpy.avi')
# cap.set(3, 4416)
# cap.set(4, 1242)
cap. set(cv2.CAP_PROP_FPS, 30)

frame_width = int(cap.get(3))
frame_height = int(cap.get(4))

print(frame_width, frame_height)

video_file = dt_string + ".avi"
out = cv2.VideoWriter(video_file, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'), 30, (frame_width, frame_height))

error_counter = 0
while True:
    ret, img = cap.read()
    if not ret:
        error_counter+=1

        if(error_counter>=100):
            break
        else:
            continue
    cv2.imshow("raw", img)
    out.write(img)

    res = cv2.waitKey(1)

    if res == ord('q'):
        break

out.release()
cap.release()