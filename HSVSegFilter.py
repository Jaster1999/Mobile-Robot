import cv2
import numpy as np

""" 0, 177 for H
0, 31 for S
68, 254 for V"""

camera = cv2.VideoCapture(2)

def filter(lower, upper):
    print("yeet")


def getrobotpos(mask):
    image, contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)


def nothing(x): # not sure why this is needed but without it slidebars dont work
    pass

cv2.namedWindow('masking') # Creating slidebar window

# Making all the slidebars to calibrate filter
cv2.createTrackbar('HLower','masking',0,255,nothing)
cv2.createTrackbar('HHigher','masking',255,255,nothing)
cv2.createTrackbar('SLower','masking',0,255,nothing)
cv2.createTrackbar('SHigher','masking',255,255,nothing)
cv2.createTrackbar('VLower','masking',0,255,nothing)
cv2.createTrackbar('VHigher','masking',255,255,nothing)

mask_map = []
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
    LowerRegion = np.array([hL,sL,vL],np.uint8)
    upperRegion = np.array([hH,sH,vH],np.uint8)
    # create mask by build a matrix of pixels which hsv values are in the specified range
    mask = cv2.inRange(hsv,LowerRegion,upperRegion)

    kernal = np.ones((1,1),"uint8")

    mask = cv2.morphologyEx(mask,cv2.MORPH_OPEN,kernal)
    mask = cv2.dilate(mask,kernal,iterations=1)

    res=cv2.bitwise_and(img, img, mask = mask)

    cv2.imshow("Masking ",res)

    inv_mask = cv2.bitwise_not(mask) 

    inv_res=cv2.bitwise_and(img, img, mask = inv_mask)
    cv2.imshow("Inv Masking",inv_res)

    blank = np.zeros((img.shape[0], img.shape[1], img.shape[2]), np.uint8)
    blank[:,0:img.shape[1]] = (255, 255, 255)

    res_2 = cv2.bitwise_and(blank, blank, mask = inv_mask)
    cv2.imshow("Inv white mask", res_2)

    mask_map = inv_mask






    if cv2.waitKey(10) & 0xFF == ord('q'):
        camera.release()
        cv2.destroyAllWindows()
        break
    
print(mask_map)
print(mask_map.shape)