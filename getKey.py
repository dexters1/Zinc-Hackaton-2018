import cv2
import time

counter = 0

while True:
    counter = counter + 1
    k = cv2.waitKey(1) 
    # press 'q' to exit
    print(k)
    time.sleep(1)
    if k == ord('q'):
        break
    