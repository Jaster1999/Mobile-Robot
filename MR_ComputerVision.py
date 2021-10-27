import cv2
import numpy as np
import math

def HSV_filter(img, lower, upper):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower, upper)
    kernal = np.ones((1,1),"uint8")
    mask = cv2.morphologyEx(mask,cv2.MORPH_OPEN,kernal)
    mask = cv2.dilate(mask,kernal,iterations=1)
    res=cv2.bitwise_and(img, img, mask = mask)
    inv_mask = cv2.bitwise_not(mask) 
    #blank = np.zeros((img.shape[0], img.shape[1], img.shape[2]), np.uint8)
    #blank[:,0:img.shape[1]] = (255, 255, 255)
    #res_2 = cv2.bitwise_and(blank, blank, mask = inv_mask)
    return inv_mask, res

def nothing(x): # function call for when sliders are moved, does nothing
    pass

def calibrate(camera): # function to calibrate various filters, you just get the values and have to manually set them in main
    cv2.namedWindow('masking') # Creating slidebar window

    # Making all the slidebars to calibrate filter
    cv2.createTrackbar('HLower','masking',0,255,nothing)
    cv2.createTrackbar('HHigher','masking',255,255,nothing)
    cv2.createTrackbar('SLower','masking',0,255,nothing)
    cv2.createTrackbar('SHigher','masking',255,255,nothing)
    cv2.createTrackbar('VLower','masking',0,255,nothing)
    cv2.createTrackbar('VHigher','masking',255,255,nothing)
    mask = []
    while(1):
        _,img = camera.read()
        img = cv2.flip(img,1)
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        hL = cv2.getTrackbarPos('HLower','masking')
        hH = cv2.getTrackbarPos('HHigher','masking')
        sL = cv2.getTrackbarPos('SLower','masking')
        sH = cv2.getTrackbarPos('SHigher','masking')
        vL = cv2.getTrackbarPos('VLower','masking')
        vH = cv2.getTrackbarPos('VHigher','masking')

        # create upper and lower hsv values
        lowerRegion = np.array([hL,sL,vL],np.uint8)
        upperRegion = np.array([hH,sH,vH],np.uint8)

        mask, res = HSV_filter(img, lowerRegion, upperRegion)

        cv2.imshow("Inv white mask", mask)
        cv2.imshow("Mask", res)
        if cv2.waitKey(10) & 0xFF == ord('d'):
            cv2.destroyAllWindows()
            break
    return (hL, hH, sL, sH, vL, vH)

clicks = []

def on_mouse(event, x, y, flags, params):
    global clicks
    if event == cv2.EVENT_LBUTTONDOWN:
        print('Seed: ' + str(x) + ', ' + str(y))
        clicks.append((y,x))

def find_point(img):
    global clicks
    cv2.namedWindow('Camera View')
    cv2.setMouseCallback('Camera View', on_mouse, 0, )
    cv2.imshow('Camera View', img)
    cv2.waitKey()
    try:
        point = clicks[0]
    except:
        point = (20, 20)
    return point


