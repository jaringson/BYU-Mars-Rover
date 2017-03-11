'''
this script opens an OpenCV window, waits for the user to click on a point on the
window, and prints the HSV value there. the purpose of it is to be run the day
of the competition for calibrating the gate_detector. it doesn't run in ROS
'''

import cv2
import numpy as np

hsv = []

def returnHSV(event, x, y, flags, param):

    if event == cv2.EVENT_LBUTTONDOWN:
        print('H='+str(hsv[y,x,0]))
        print('S='+str(hsv[y,x,1]))
        print('V='+str(hsv[y,x,2]))
        print('----------')

cap = cv2.VideoCapture(0)

# img = cv2.imread('images/campus.jpg')
cv2.namedWindow("Ball")
cv2.setMouseCallback("Ball",returnHSV)

while True:
    ret,frame = cap.read()

    changed_img = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    hsv=changed_img
    cv2.imshow("Ball",frame)
    c = cv2.waitKey(1)
    if c == 27:
        break

cap.release()
cv2.destroyAllWindows()
