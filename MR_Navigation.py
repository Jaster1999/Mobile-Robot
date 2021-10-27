import cv2
import numpy as np
import math
import heapq

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

def getpos(mask):
    contours, hierarchy = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    # calculate moments for each contour
    areas = [cv2.contourArea(c) for c in contours]
    max_index = np.argmax(areas)
    c = contours[max_index]
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
    elif(xdif == 0 and ydif<0):
        angle_rad = math.pi
    elif(ydif == 0 and xdif > 0):
        angle_rad = math.pi/2
    elif(ydif == 0 and xdif < 0):
        angle_rad = -(math.pi/2)
    #if(angle_rad<0):
     #   angle_rad += 2*math.pi
    return math.degrees(angle_rad)

def generate_command(path, angle, current_pos):
    msg = ' '
    try:
        if(len(path) == 0):
            return msg
        else:
            if angle>10 and angle <= 180:
                msg = "9"
            elif angle<350 and angle > 180:
                msg = "7"
            elif path[0][0] - current_pos[0] >= 1: # was right now forward
                msg = "W"
            elif path[0][0] - current_pos[0] <= -1: # was left now back
                msg = "S"
            elif path[0][1] - current_pos[1] >= 1: # was back now right
                msg = "D"
            elif path[0][1] - current_pos[1] <= -1: # was forward now left
                msg = "A"
            else:
                msg = "0"
    except:
        msg = ' '
    return msg