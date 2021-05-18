import serial
import time
from rich import print
from rich.progress import Progress
import math
com_port = 'COM6'

def get_angle(robot_center, robot_front):
    angle = 0
    dx = robot_front[0] - robot_center[0]
    dy = robot_front[1] - robot_center[1]
    angle = -math.atan2(-dx, dy) #returns angle in radians
    if angle<0:
        angle = 360+angle
    angle = math.degrees(angle)
    return angle 
def generate_command(path, angle):
    msg = ''
    if(len(path) == 1):
        return msg
    else:
        if angle>10 and angle <= 180:
            msg = "7"
        elif angle<350 and angle > 180:
            msg = "9"
        elif path[1][0] - path[0][0] >= 1:
            msg = "D"
        elif path[1][0] - path[0][0] <= -1:
            msg = "A"
        elif path[1][1] - path[0][1] >= 1:
            msg = "W"
        elif path[1][1] - path[0][1] <= -1:
            msg = "S"
        else:
            msg = "0"
    return msg

def main():
    path = [(0, 0), (0, 1), (0, 2), (0, 3), (0, 4), (0, 5), (0, 6), (0, 7), (0, 8), (0, 9), (0, 10), (0, 11), (0, 12), (0, 13), (0, 14), (0, 15), (0, 16), (0, 17), (0, 18), (0, 19), (0, 20), (0, 21), (0, 22), (0, 23), (0, 24), (0, 25), (0, 26), (0, 27), (0, 28), (0, 29), (0, 30), (0, 31), (0, 32), (0, 33), (0, 34), (0, 35), (0, 36), (0, 37), (0, 38), (0, 39), (0, 40), (0, 41), (0, 42), (0, 43), (0, 44), (0, 45), (0, 46), (0, 47), (0, 48), (0, 49), (0, 50), (0, 51), (0, 52), (0, 53), (0, 54), (0, 55), (0, 56), (0, 57), (0, 58), (0, 59), (0, 60), (0, 61), (0, 62), (0, 63), (0, 64), (0, 65), (0, 66), (0, 67), (0, 68), (0, 69), (0, 70), (0, 71), (0, 72), (0, 73), (0, 74), (0, 75), (0, 76), (0, 77), (0, 78), (0, 79), (0, 80), (0, 81), (0, 82), (0, 83), (0, 84), (0, 85), (1, 86), (2, 87), (2, 88), (2, 89), (2, 90), (2, 91), (2, 92), (2, 93), (2, 94), (2, 95), (2, 96), (2, 97), (2, 98), (2, 99), (2, 100), (1, 101), (0, 102), (0, 103), (0, 104), (0, 105), (0, 106), (0, 107), (0, 108), (0, 109), (0, 110), (0, 111), (0, 112), (0, 113), (1, 114), (2, 114), (3, 114), (4, 114), (5, 114), (6, 114), (7, 113), (8, 113), (9, 114), (10, 115), (11, 116), (12, 117), (13, 118), (14, 118), (15, 118), (16, 118), (17, 118), (18, 118), (19, 118), (20, 118), (21, 118), (22, 118), (23, 118), (24, 118), (25, 119), (26, 120), (27, 121), (28, 122), (29, 122), (30, 122), (31, 122), (32, 122), (33, 122), (34, 122), (35, 122), (36, 123), (37, 124), (38, 125), (39, 125), (40, 125), (41, 125), (42, 125), (43, 125), (44, 125), (45, 124), (46, 124), (47, 124), (48, 124), (49, 124), (50, 124), (51, 124), (52, 124), (53, 124), (54, 124), (55, 124), (56, 124), (57, 124), (58, 124), (59, 124), (60, 124), (61, 124), (62, 124), (63, 124), (64, 124), (65, 124), (66, 124), (67, 124), (68, 124), (69, 124), (70, 124), (71, 124), (72, 124), (73, 124), (74, 124), (75, 124), (76, 124), (77, 124), (78, 124), (79, 124), (80, 124), (81, 124), (82, 124), (83, 124), (84, 124), (85, 124), (86, 124), (87, 124), (88, 124), (89, 124), (90, 124), (91, 124), (92, 124), (93, 124), (94, 124), (95, 124), (96, 124), (97, 124), (98, 124), (99, 124), (100, 124), (101, 124), (102, 124), (103, 124), (104, 124), (105, 124), (106, 124), (107, 124), (108, 124), (109, 124), (110, 124), (111, 124), (112, 124), (113, 124), (114, 124), (115, 124), (116, 124), (117, 124), (118, 124), (119, 124), (120, 124), (121, 124), (122, 125), (123, 125), (124, 126), (125, 126), (126, 126), (127, 127)]
    angle = 15
    port = serial.Serial(com_port, baudrate=115200)
    if (port.is_open == False):
        port.open()
        print("[green]port open [/]")

    if port.is_open:
        print(port.is_open)
        print("[green]port open[/]\n")
        with Progress() as progress:
            task1 = progress.add_task("[red]Waiting for MCU...[/]", total = 10)
            while not progress.finished:
                time.sleep(1)
                progress.update(task1, advance=1)
        print("[green]Waiting complete![/]")
        #time.sleep(10)   #wait needed from port to open properly and MCU to be ready

        length_of_path = len(path)
        for i in range(length_of_path):
            cmd = generate_command(path, angle)
            print(cmd)
            port.write(bytes(cmd, 'UTF-8'))
            port.write(b'\n') #add a carriage return character
            if cmd[0] in ["A", "S", "D", "W"]:
                path.pop(0)
                angle += 5
            elif cmd[0] == '9':
                angle += 15
            elif cmd[0] == '7':
                angle -= 15
            time.sleep(1)
        
    else:
        print("[red]oop[/][yellow], couldnt open port[/]\n")    

    
    port.close()
    print("port closed")
    return 0

if __name__ == "__main__":
    main()

