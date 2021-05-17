import cv2
import numpy as np
import math
import heapq
import matplotlib.pyplot as plt
from matplotlib.pyplot import figure

""" 0, 177 for H
0, 31 for S
68, 254 for V"""


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

def calibrate(img): # function to calibrate various filters, you just get the values and have to manually set them in main
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
    angle_rad = math.atan(xdif/ydif)
    return math.degrees(angle_rad)


# heuristic function for path scoring
def heuristic(a, b):
    return np.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)

# path finding function
def astar(array, start, goal):
    neighbors = [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]
    close_set = set()
    came_from = {}
    gscore = {start:0}
    fscore = {start:heuristic(start, goal)}
    oheap = []
    heapq.heappush(oheap, (fscore[start], start))
 
    while oheap:
        current = heapq.heappop(oheap)[1]
        if current == goal:
            data = []
            while current in came_from:
                data.append(current)
                current = came_from[current]
            return data
        close_set.add(current)
        for i, j in neighbors:
            neighbor = current[0] + i, current[1] + j
            tentative_g_score = gscore[current] + heuristic(current, neighbor)
            if 0 <= neighbor[0] < array.shape[0]:
                if 0 <= neighbor[1] < array.shape[1]:                
                    if array[neighbor[0]][neighbor[1]] == 1:
                        continue
                else:
                    # array bound y walls
                    continue
            else:
                # array bound x walls
                continue
            if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                continue
            if  tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1]for i in oheap]:
                came_from[neighbor] = current
                gscore[neighbor] = tentative_g_score
                fscore[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                heapq.heappush(oheap, (fscore[neighbor], neighbor))
    return False


def main():
    camera = cv2.VideoCapture(2)
    _,img = camera.read()
    img = cv2.flip(img,1)

    # Calibrate the filters
    print("Calibrate?")
    resp = input("y/n?")
    if(resp.lower() == 'y'):
        print("Background filter", calibrate(img))
        print("Robot filter", calibrate(img))

    # end goal coords
    goal = (63,63)

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

        # create dot filter, dot is the blue dot on the robot
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

        # take the robot out as an object in the background mask
        combined_mask = cv2.subtract(dilated_mask, dilated_rob_mask) 
        cv2.imshow("Combined mask", combined_mask)

        # find the robot's centre and the blue dot and calculate the robots orientation
        robot_centre = getpos(robot_mask)
        cv2.circle(img, robot_centre, 5, (255, 255, 255), -1)
        cv2.putText(img, "centroid", (robot_centre[0] + 25, robot_centre[1] + 25),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        cv2.imshow("Image", img)
        dot_centre = getpos(dot_mask) 
        current_angle = getangle(robot_centre, dot_centre) # finding the robots angle relative to the camera frame coordinates.

        # produce a grid for the astar pathfinder
        resized_mask = cv2.resize(combined_mask, None, fx=0.1, fy=0.1, interpolation = cv2.INTER_CUBIC) # scaled to one tenth the original image size
        scaled_robotcoords = (int(robot_centre[0]/20), int(robot_centre[1]/20)) # transforming robot coords into grid coord system.
        print(resized_mask)
        print(resized_mask.shape)
        resized_mask[resized_mask!=0]=1 # turning grid into 1's and 0's
        print(resized_mask)

        # finding the route, remember to set a real end goal
        route = astar(resized_mask, scaled_robotcoords, goal)

        if cv2.waitKey(10) & 0xFF == ord('q'):
            camera.release()
            cv2.destroyAllWindows()
            break

if __name__ == "__main__":
    main()