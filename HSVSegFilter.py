'''import cv2
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
from matplotlib import colors
from skimage.io import imread

def nothing():
    pass

img = imread("/home/justin/Documents/testimg.jpg")
plt.imshow(img)

cv2.namedWindow('Thresholding')

cv2.createTrackbar('lowH','Thresholding',0,255,nothing)
cv2.createTrackbar('highH','Thresholding',255,255,nothing)
cv2.createTrackbar('lowS','Thresholding',0,255,nothing)
cv2.createTrackbar('highS','Thresholding',255,255,nothing)
cv2.createTrackbar('lowV','Thresholding',0,255,nothing)
cv2.createTrackbar('highV','Thresholding',255,255,nothing)

while True:
    # grab the frame
    #ret, frame = cap.read()

    # get trackbar positions
    ilowH = cv2.getTrackbarPos('lowH', 'Thresholding')
    ihighH = cv2.getTrackbarPos('highH', 'Thresholding')
    ilowS = cv2.getTrackbarPos('lowS', 'Thresholding')
    ihighS = cv2.getTrackbarPos('highS', 'Thresholding')
    ilowV = cv2.getTrackbarPos('lowV', 'Thresholding')
    ihighV = cv2.getTrackbarPos('highV', 'Thresholding')

    hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
    lower_hsv = np.array([ilowH, ilowS, ilowV])
    higher_hsv = np.array([ihighH, ihighS, ihighV])
    mask = cv2.inRange(hsv, lower_hsv, higher_hsv)

    frame = cv2.bitwise_and(img, img, mask=mask)

    # show thresholded image
    cv2.imshow('image', frame)
    k = cv2.waitKey(1000) & 0xFF # large wait time to remove freezing
    if k == 113 or k == 27:
        break

    cv2.destroyAllWindows()'''


import cv2
import numpy as np

""" 0, 177 for H
0, 31 for S
68, 254 for V"""

camera = cv2.VideoCapture(2)

def nothing(x):
    pass

cv2.namedWindow('marking')

cv2.createTrackbar('H Lower','marking',0,255,nothing)
cv2.createTrackbar('H Higher','marking',255,255,nothing)
cv2.createTrackbar('S Lower','marking',0,255,nothing)
cv2.createTrackbar('S Higher','marking',255,255,nothing)
cv2.createTrackbar('V Lower','marking',0,255,nothing)
cv2.createTrackbar('V Higher','marking',255,255,nothing)


while(1):
    _,img = camera.read()
    img = cv2.flip(img,1)

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    hL = cv2.getTrackbarPos('H Lower','marking')
    hH = cv2.getTrackbarPos('H Higher','marking')
    sL = cv2.getTrackbarPos('S Lower','marking')
    sH = cv2.getTrackbarPos('S Higher','marking')
    vL = cv2.getTrackbarPos('V Lower','marking')
    vH = cv2.getTrackbarPos('V Higher','marking')

    LowerRegion = np.array([hL,sL,vL],np.uint8)
    upperRegion = np.array([hH,sH,vH],np.uint8)

    redObject = cv2.inRange(hsv,LowerRegion,upperRegion)

    kernal = np.ones((1,1),"uint8")

    red = cv2.morphologyEx(redObject,cv2.MORPH_OPEN,kernal)
    red = cv2.dilate(red,kernal,iterations=1)

    res1=cv2.bitwise_and(img, img, mask = red)


    cv2.imshow("Masking ",res1)

    if cv2.waitKey(10) & 0xFF == ord('q'):
        camera.release()
        cv2.destroyAllWindows()
        break
    