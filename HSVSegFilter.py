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


def getpos(mask):
    contours, hierarchy = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
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
    if(xdif or ydif != 0):
        angle_rad = -math.atan2(-xdif, ydif)
    elif(xdif == 0 and ydif > 0):
        angle_rad = 0
    elif(xidf == 0 and ydif<0):
        angle_rad = math.pi
    elif(ydif == 0 and xdif > 0):
        angle_rad = math.pi/2
    elif(ydif == 0 and xdif < 0):
        angle_rad = -(math.pi/2)
    #if(angle_rad<0):
     #   angle_rad += 2*math.pi
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

    # Calibrate the filters
    print("Calibrate?")
    resp = input("y/n?")
    if(resp.lower() == 'y'):
        print("Background filter", calibrate(camera))
        print("Robot filter", calibrate(camera))

    # end goal coords
    goal = (63,63)

    while(1):
        _,img = camera.read()
        img = cv2.flip(img,1)

        # create dot filter, dot is the blue dot on the robot
        lowerRegionBackground = np.array([99, 108, 120],np.uint8)
        upperRegionBackground = np.array([111, 255, 187],np.uint8)
        dot_mask, dot_res = HSV_filter(img, lowerRegionBackground, upperRegionBackground)
        dot_res=[]
        inv_dot_mask = cv2.bitwise_not(dot_mask) 
        cv2.imshow("Dilation adj dot_mask", inv_dot_mask)

        # create robot filter
        lowerRegionBackground = np.array([0, 119, 131],np.uint8)
        upperRegionBackground = np.array([32, 255, 225],np.uint8)
        robot_mask, robot_res = HSV_filter(img, lowerRegionBackground, upperRegionBackground)
        robot_res=[]
        inv_rob_mask = cv2.bitwise_not(robot_mask) 
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        dot_plus_robot_mask = cv2.dilate(cv2.add(inv_rob_mask, inv_dot_mask), kernel, iterations=1)
        kernel2 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (31, 31))
        eroded_rob_mask = cv2.erode(inv_rob_mask, (7, 7), iterations=3)
        dilated_rob_mask = cv2.dilate(eroded_rob_mask, kernel2, iterations=1)
        cv2.imshow("Dilation adj rob_mask", dilated_rob_mask)

        # create Background filter
        lowerRegionBackground = np.array([0, 0, 69],np.uint8)
        upperRegionBackground = np.array([177, 42, 255],np.uint8)
        mask, res = HSV_filter(img, lowerRegionBackground, upperRegionBackground)
        combined_mask = cv2.subtract(mask, dot_plus_robot_mask) 
        #inv_mask = cv2.bitwise_not(combined_mask)
        eroded_mask = cv2.erode(combined_mask, (7, 7), iterations=3)
        dilated_mask = cv2.dilate(eroded_mask, kernel, iterations=5)
        cv2.imshow("Dilation adj mask", dilated_mask)





        # find the robot's centre and the blue dot and calculate the robots orientation
        try:
            robot_centre = getpos(dilated_rob_mask)
            cv2.circle(img, robot_centre, 2, (255, 255, 255), -1)
            cv2.putText(img, "centroid", (robot_centre[0] + 25, robot_centre[1] + 25),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            dot_centre = getpos(inv_dot_mask) 
            cv2.circle(img, dot_centre, 2, (0, 255, 255), -1)
            cv2.putText(img, "dot centroid", (dot_centre[0] - 25, dot_centre[1] - 25),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            current_angle = getangle(robot_centre, dot_centre) # finding the robots angle relative to the camera frame coordinates.
            #print(dot_centre, robot_centre)
        except:
            #print("Robot lost")
            current_angle = 0
        cv2.imshow("dots", img)
        print(current_angle)

        # produce a grid for the astar pathfinder
        resized_mask = cv2.resize(dilated_mask, None, fx=0.05, fy=0.05, interpolation = cv2.INTER_CUBIC) # scaled to one tenth the original image size
        #scaled_robotcoords = (int(robot_centre[0]/20), int(robot_centre[1]/20)) # transforming robot coords into grid coord system.
        resized_mask[resized_mask!=0]=1 # turning grid into 1's and 0's
        #print(resized_mask)
        for line in resized_mask:
            line = str(line)
            new_line = ''
            for char in line.split():
                new_line = new_line+char+", "
            #print(new_line)
        #print(resized_mask.shape)

        # finding the route, remember to set a real end goal
        #route = astar(resized_mask, scaled_robotcoords, goal)

        if cv2.waitKey(10) & 0xFF == ord('q'):
            camera.release()
            cv2.destroyAllWindows()
            break

if __name__ == "__main__":
    main()