import cv2
import numpy as np
import math

""" 0, 177 for H
0, 31 for S
68, 254 for V"""

camera = cv2.VideoCapture(2)
#cap.set(cv.CV_CAP_PROP_FRAME_WIDTH, 800)
#cap.set(cv.CV_CAP_PROP_FRAME_HEIGHT, 600)

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

def nothing(x): # function call for when sliders are moved
    pass

def calibrate(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
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
            camera.release()
            cv2.destroyAllWindows()
            break
    return (hL, hH, sL, sH, vL, vH)

#def get_angle():

def getpos(mask):
    contours, hierarchy = cv2.findContours(dilated_rob_mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    # calculate moments for each contour
    c = contours[0]
    M = cv2.moments(c)
    # calculate x,y coordinate of center
    cX = int(M["m10"] / M["m00"])
    cY = int(M["m01"] / M["m00"])
    return (cX, cY)

def getangle(pos1, pos2):
    xdif = pos2[0]-pos1[0]
    ydif = pos2[1]-pos1[1]
    angle_rad = math.atan()

    
    return math.degrees(angle_rad)

def find_paths_breadthfirst(labelled_map, position, destination):
    # yeah
    print("Yes")


_,img = camera.read()
img = cv2.flip(img,1)

# Calibrate the filters
print("Calibrate?")
resp = input("y/n?")
if(resp.lower() == 'y'):
    print("Background filter", calibrate(img))
    print("Robot filter", calibrate(img))

while(1):
    _,img = camera.read()
    img = cv2.flip(img,1)

    # create Background filter
    lowerRegionBackground = np.array([0, 0, 68],np.uint8)
    upperRegionBackground = np.array([177, 31, 255],np.uint8)
    mask, res = HSV_filter(img, lowerRegionBackground, upperRegionBackground)
    eroded_mask = cv2.erode(mask, (7, 7), iterations=3)
    dilated_mask = cv2.dilate(eroded_mask, (61, 61), iterations=31)
    cv2.imshow("Dilation adj mask", dilated_mask)

    # create dot filter
    lowerRegionBackground = np.array([0, 119, 131],np.uint8)
    upperRegionBackground = np.array([32, 255, 225],np.uint8)
    dot_mask, dot_res = HSV_filter(img, lowerRegionBackground, upperRegionBackground)
    dot_res=[]
    inv_dot_mask = cv2.bitwise_not(dot_mask) 
    cv2.imshow("Dilation adj dot_mask", dot_mask)

    # create robot filter
    lowerRegionBackground = np.array([0, 119, 131],np.uint8)
    upperRegionBackground = np.array([32, 255, 225],np.uint8)
    robot_mask, robot_res = HSV_filter(img, lowerRegionBackground, upperRegionBackground)
    robot_res=[]
    inv_rob_mask = cv2.bitwise_not(robot_mask) 
    dot_plus_robot_mask = cv2.add(inv_rob_mask, inv_dot_mask)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (31, 31))
    eroded_rob_mask = cv2.erode(inv_rob_mask, (7, 7), iterations=3)
    dilated_rob_mask = cv2.dilate(eroded_rob_mask, kernel, iterations=1)
    cv2.imshow("Dilation adj rob_mask", dilated_rob_mask)


    combined_mask = cv2.subtract(dilated_mask, dilated_rob_mask) 
    cv2.imshow("Combined mask", combined_mask)

    robot_centre = getpos(robot_mask)
    cv2.circle(img, robot_centre, 5, (255, 255, 255), -1)
    cv2.putText(img, "centroid", (robot_centre[0] + 25, robot_centre[1] + 25),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
    # display the image
    cv2.imshow("Image", img)

    dot_centre = getpos(dot_mask)
    #current_angle = getangle(robot_centre, dot_centre)

    resized_mask = cv2.resize(combined_mask, None, fx=0.1, fy=0.1, interpolation = cv2.INTER_CUBIC)
    scaled_robotcoords = (int(robot_centre[0]/20), int(robot_centre[1]/20))

    print(resized_mask)
    print(resized_mask.shape)
    resized_mask[resized_mask==255]=1
    #mask_map = cv2.resize(mask, (128, 128), cv2.INTER_LINEAR)

    # cv2.imshow("Grid", mask_map)
    # labelled_map = []
    # for x in range(len(mask_map)):
    #     for i in range(len(mask_map[0])):
    #         if(mask_map):
    #             print("Processing")

    if cv2.waitKey(10) & 0xFF == ord('q'):
        camera.release()
        cv2.destroyAllWindows()
        break
    
print(mask_map)
print(mask_map.shape)