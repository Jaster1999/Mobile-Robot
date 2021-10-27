import cv2
from cv2 import data
import numpy as np
import PySimpleGUI as sg
import os
import matplotlib.pyplot as plt
import math
import imutils
import time
import csv
import serial
import threading
from rich import print
from rich.progress import Progress
import Manipulatorkinematics as MK
import MR_ComputerVision as CV
import MR_Navigation as Nav



def convert_image_thread():
        global imgbytes
        global run_thread1
        global img
        while run_thread1:
                imgbytes = cv2.imencode(".png", img)[1].tobytes()
        return


img = np.zeros((1280, 720, 3))
imgbytes = cv2.imencode(".png", img)[1].tobytes()
run_thread1 = True
imgthread = threading.Thread(target=convert_image_thread)

def main():
    camera = cv2.VideoCapture(3, cv2.CAP_DSHOW)
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
    sg.theme('default1')

    arm = MK.Manipulator()
    arm.setDH_parameters(125,360,120,170,80,-67,-67,0)
    
    global run_thread1 
    global imgbytes
    global img
    imgthread.start()

    #load images into elements
    image_elem1 = sg.Image(key="rgb", data=imgbytes)
    image_elem2 = sg.Image(key="mask", data=imgbytes)
    


    # 1st attributes list
    # Handles image processing settings
    col_11 =[
            [sg.Button("Start")],
            [sg.Button(button_text="Calibrate Obstacle Filter"), sg.Text(key="obsfiltervalues", size=(20, 1))],
            [sg.Button(button_text="Calibrate Robot Filter"), sg.Text(key="robotfiltervalues", size=(20, 1))],
            [sg.Button(button_text="Calibrate Dot Filter"), sg.Text(key="dotfiltervalues", size=(20, 1))],
            [sg.Button(button_text="Set Goal"), sg.Text(key="goalvalues", size=(20, 1))]]

    # 2nd attributes list
    # Handles xyz positioning for manipulator
    col_12 =[
            [sg.Text("Manipulator Position:")],
            [sg.Text("X:"), sg.Text(key="x", size=(10, 1)), sg.Text("Y:"), sg.Text(key="y", size=(10, 1)), sg.Text("Z:"), sg.Text(key="z", size=(10, 1))],
            [sg.Text("Manipulator Joint Angles and positions:")],
            [sg.Text("Joint1:"), sg.Text(key="joint1", size=(10, 1)), sg.Text("Joint2:"), sg.Text(key="joint2", size=(10, 1)), sg.Text("Joint3:"), sg.Text(key="joint3", size=(10, 1)), sg.Text("Joint4:"), sg.Text(key="joint4", size=(10, 1))],
            [sg.Text("Set Manipulator Position:")],
            [sg.Text("Set X: ", size=(10, 1)), sg.Input(size=(10, 1), key="setx", default_text = "0"), sg.Text("Set Y: ", size=(10, 1)), sg.Input(size=(10, 1), key="sety", default_text = "0"), sg.Text("Set Z: ", size=(10, 1)), sg.Input(size=(10, 1), key="setz", default_text = "0")],
            [sg.Text("Orientation"), sg.Input(size=(10, 1), key="orientation", default_text = "0")],
            [sg.Button("Confirm Coordinates"), sg.Button("Gripper")],
            [sg.Button("Stop"), sg.Button("Home")]]


    # 3rd attributes list
    col_13 =[
            [sg.Text("Placeholder")],
            #[sg.Text("X:"), sg.Text(key="x", size=(10, 1)), sg.Text("Y:"), sg.Text(key="y", size=(10, 1)), sg.Text("Z:"), sg.Text(key="z", size=(10, 1))],
            #[sg.Button("Stop"), sg.Button("Home"), sg.Text("StepDistance(mm):"), sg.Input(size=(10, 1), key="steps"), sg.Button("Confirm Distance")],
            #[sg.Button("Forward X", size=(10, 1)), sg.Button("Reverse X", size=(10, 1)), sg.Button("Preset A", size=(10, 1))],
            #[sg.Button("Forward Y", size=(10, 1)), sg.Button("Reverse Y", size=(10, 1))],
            #[sg.Button("Z Down", size=(10, 1)), sg.Button("Z Up", size=(10, 1))]
            ]

    #images column
    col_2_1 = [
            [sg.Frame(layout=[[image_elem1]], vertical_alignment = 'c',title='')]
            ]
    col_2_2 = [
            [sg.Frame(layout=[[image_elem2]], vertical_alignment = 'c',title='')],
            ]

    #attributes column
    col_1 = [
            [sg.Frame(layout=[[sg.Column(col_11, vertical_alignment = 't')]], vertical_alignment = 't',title='')],
            [sg.Frame(layout=[[sg.Column(col_12, vertical_alignment = 'c')]], vertical_alignment = 'c',title='')],
            [sg.Frame(layout=[[sg.Column(col_13, vertical_alignment = 'c')]], vertical_alignment = 'c',title='')]]

    
    tabgrp = [[sg.Column(col_1), sg.TabGroup([[sg.Tab("Navigation", col_2_1), sg.Tab("Obstacles", col_2_2)]])]]

    #layout = [[sg.Column(col_1), sg.Column(col_2_1)]]
    # Create the Window
    w, h = sg.Window.get_screen_size()
    window = sg.Window('Manipulator and Mobile Robot UI', tabgrp, location=(0,0), resizable=True, size=(w,int(0.95*h)), keep_on_top=False)#.Finalize()
    #window.Maximize()

    com_port = input("type comport: ")
    goal = []
    gripperopen = True
    toggled = False
    navigate = False
    goal_set = False
    lowerRegionBackground = np.array([99, 109, 100],np.uint8)
    upperRegionBackground = np.array([109, 255, 184],np.uint8)
    lowerRegionRobot = np.array([0, 142, 131],np.uint8)
    upperRegionRobot = np.array([32, 255, 225],np.uint8)
    lowerRegionDot = np.array([0, 0, 69],np.uint8)
    upperRegionDot = np.array([177, 42, 255],np.uint8)

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

    com_port2 = input("comport 2: ")
    port2 = serial.Serial(com_port2, baudrate=115200)
    if (port2.is_open == False):
        port2.open()


    # Event Loop to process "events" and get the "values" of the inputs
    while True:
        event, values = window.read(timeout = 0.1)
        
        if event == sg.WIN_CLOSED:
            time.sleep(10)
            break
        if event == "Start":
            navigate = True
        if event =="Calibrate Obstacle Filter":
                filter_values = CV.calibrate(camera)
                window["obsfiltervalues"].update(str(filter_values))
                lowerRegionBackground = np.array([filter_values[0], filter_values[2], filter_values[5]],np.uint8)
        if event =="Calibrate Robot Filter":
                filter_values = CV.calibrate(camera)
                window["robotfiltervalues"].update(str(filter_values))
                lowerRegionBackground = np.array([filter_values[0], filter_values[2], filter_values[5]],np.uint8)
        if event =="Calibrate Dor Filter":
                filter_values = CV.calibrate(camera)
                window["dotfiltervalues"].update(str(filter_values))
                lowerRegionBackground = np.array([filter_values[0], filter_values[2], filter_values[5]],np.uint8)
        if event == "Set Goal":
                point = []
                set_point = CV.find_point(img)
                goal = (int(set_point[1]/20), int(set_point[0]/20))
                print(goal)
        elif event == "Stop":
                print("Manipulator Stopping")
                port2.write(bytes('#', 'UTF-8'))
                port2.flush()
        elif event == "Home":
                port2.write(bytes('h\n', 'UTF-8'))
                port2.flush()
                print("Home Command sent")
        elif event == "Confirm Coordinates":
                print("Moving to defined coordinates")
                x = int(values["setx"])
                y = int(values["sety"])
                z = int(values["setz"])
                window["x"].update(str(x))
                window["y"].update(str(y))
                window["z"].update(str(z))
                Orientation = int(values["orientation"])
                Orientation = arm.AngleToOrientation(Orientation)
                Position = (x, y, z)
                
                Joint1, Joint2, Joint3, Joint4 = arm.InvKine(Position, Orientation)
                print(Joint1)
                Joint1 = arm.DistanceValidator(Joint1)
                Joint2 = arm.AngleValidator(Joint2)
                Joint3 = arm.AngleValidator(Joint3)
                Joint4 = arm.AngleValidator(Joint4)
                #convert invkine (-180º to 180º) angle to a servo angle (0º to 180º)
                print("Servo 2 Move to: "+str(Joint2)+"º")
                print("Servo 3 Move to: "+str(Joint3)+"º")
                print("Servo 4 Move to: "+str(Joint4)+"º")
                # Need to round to mm as the stepper firmware expects an int mm
                print("Stepper 1 Move to: "+str(round(Joint1))+"mm")

                port2.write(bytes('2'+str(Joint2)+'\n', 'UTF-8'))
                time.sleep(0.01)
                port2.write(bytes('3'+str(Joint3)+'\n', 'UTF-8'))
                time.sleep(0.01)
                port2.write(bytes('4'+str(Joint4)+'\n', 'UTF-8'))
                time.sleep(0.01)
                port2.write(bytes('Z'+str(round(Joint1))+'\n', 'UTF-8'))
                port2.flush()

        elif event == "Gripper":
                gripperopen = not gripperopen
                toggled = True

        if(gripperopen and toggled):
                port2.write(bytes('G0\n', 'UTF-8'))
                port2.flush()
                print("Opening Gripper")
                toggled = not toggled
        elif(not gripperopen and toggled):
                port2.write(bytes('G180\n', 'UTF-8'))
                port2.flush()
                print("Closing Gripper")
                toggled = not toggled




        if navigate and goal_set:
                _,img = camera.read()
                img = cv2.flip(img,1)

                dot_mask, dot_res = CV.HSV_filter(img, lowerRegionDot, upperRegionDot)
                kernel4 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
                dot_close = cv2.morphologyEx(dot_mask, cv2.cv2.MORPH_CLOSE, kernel4, iterations=2)
                dot_res=[]
                inv_dot_mask = cv2.bitwise_not(dot_close) 
                cv2.imshow("Dilation adj dot_mask", cv2.dilate(inv_dot_mask, kernel4, iterations=5))

                # create robot filter
                robot_mask, robot_res = CV.HSV_filter(img, lowerRegionRobot, upperRegionRobot)
                robot_res=[]
                inv_rob_mask = cv2.bitwise_not(robot_mask) 
                inv_robot_mask = cv2.morphologyEx(inv_rob_mask, cv2.MORPH_CLOSE, kernel4, iterations=3)
                kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
                dot_plus_robot_mask = cv2.dilate(cv2.add(inv_robot_mask, cv2.dilate(inv_dot_mask, kernel, iterations=5)), kernel, iterations=5)
                kernel2 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (31, 31))
                eroded_rob_mask = cv2.erode(inv_rob_mask, (7, 7), iterations=0)
                dilated_rob_mask = cv2.dilate(eroded_rob_mask, kernel2, iterations=1)
                cv2.imshow("Dilation adj rob_mask", dilated_rob_mask)

                # create Background filter
                mask, res = CV.HSV_filter(img, lowerRegionBackground, upperRegionBackground)
                combined_mask = cv2.subtract(mask, dot_plus_robot_mask) 
                #inv_mask = cv2.bitwise_not(combined_mask)
                eroded_mask = cv2.erode(combined_mask, (7, 7), iterations=3)
                kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))
                dilated_mask = cv2.dilate(eroded_mask, kernel, iterations=4)
                cv2.imshow("Dilation adj mask", dilated_mask)

                # find the robot's centre and the blue dot and calculate the robots orientation
                try:
                        robot_centre = Nav.getpos(dilated_rob_mask)
                        cv2.circle(img, robot_centre, 2, (255, 255, 255), -1)
                        cv2.putText(img, "centroid", (robot_centre[0] + 25, robot_centre[1] + 25),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                        dot_centre = Nav.getpos(inv_dot_mask) 
                        cv2.circle(img, dot_centre, 2, (0, 255, 255), -1)
                        cv2.putText(img, "dot centroid", (dot_centre[0] - 25, dot_centre[1] - 25),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                        current_angle = Nav.getangle(robot_centre, dot_centre) # finding the robots angle relative to the camera frame coordinates.
                        #print(dot_centre, robot_centre)
                except:
                        print("Robot lost")
                        current_angle = 0
                
                # produce a grid for the astar pathfinder
                resized_mask = cv2.resize(dilated_mask, None, fx=0.025, fy=0.025, interpolation = cv2.INTER_CUBIC) # scaled to one tenth the original image size
                resized_mask[resized_mask!=0]=1 # turning grid into 1's and 0's
                #print(resized_mask)
                for line in resized_mask:
                        line = str(line)
                        new_line = ''
                        for char in line.split():
                                new_line = new_line+char+", "
                        print(new_line)
                print(resized_mask.shape)
                print(goal)

                 # transforming robot coords into grid coord system.
                scaled_robotcoords = (int(robot_centre[1]/40), int(robot_centre[0]/40))
                print(scaled_robotcoords)
                #print(scaled_robotcoords)
                # finding the route, remember to set a real end goal
                route = Nav.astar(resized_mask, scaled_robotcoords, goal)
                try:
                        route = route[::-1]
                except:
                        route = scaled_robotcoords
                        print("No route found")
                # print(route)
                try:
                        for coord in route:
                                #print(coord)
                                cv2.circle(img, (coord[1]*40, coord[0]*40), 3, (255, 0, 0), -1)
                except:
                        pass
                cv2.circle(img, (goal[1]*40, goal[0]*40), 3, (255, 0, 0), -1)
                cv2.imshow("dots", img)

                path = route
                angle = current_angle
                current_pos = scaled_robotcoords

                if port.is_open:
                        cmd = Nav.generate_command(path, angle, current_pos)
                        port.write(bytes(cmd, 'UTF-8'))
                        port.write(b'\n') #add a carriage return character
                        print(cmd)
                        time.sleep(1)
                else:
                        print("[red]oop[/][yellow], couldnt open port[/]\n")    

                window["rgb"].update(data=imgbytes)

                if cv2.waitKey(10) & 0xFF == ord('q'):
                        camera.release()
                        cv2.destroyAllWindows()
                        break

    run_thread1 = False
    time.sleep(0.5)
    window.close()
    port.close()
    

if __name__ == "__main__":
    main()