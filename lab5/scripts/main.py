import cmath
from math import atan2
from inv_kinematics import getJointValues, getPerpendicularOrientation 
from parser import getTrajectoryFromTextFile
from phantom import *
import numpy as np
import time
import os 
from mapValue import * 



file_names = ["circle.txt", "custom_part.txt", "d_letter.txt",
                "inner_ring.txt", "outter_ring.txt", "line123.txt",
                "point1to5.txt", "s_letter.txt", "triangle.txt","load.txt"]

drawing_distance_From_Plane = 109
moving_distance_From_Plane = 139

HOME = np.array([-85,0,-90,10]) #Our HOME position in absolute coordinates

open_Gripper_Dig_Value = 515
closed_Gripper_Dig_Value = 110
closed_Gripper_Angle_Value = -106 #Degrees
open_Gripper_Angle_Value = 0 #Degrees





def print_state(filename, coord):
    print("Routine: {}. End effector: X: {} Y: {} Z: {} (mm)"
            .format(filename, coord[0], coord[1], coord[2]))


def draw(filename):

    trajectory_filename = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),"trajectories",filename)
    trajectory = getTrajectoryFromTextFile(trajectory_filename)

    initializeTrajectory()

    for coord in trajectory:

        pose = getPerpendicularOrientation(coord)

        pose[0,3] = coord[0]
        pose[1,3] = coord[1]

        durationScale = 1

        if(coord[2] == 1):
            pose[2,3] = moving_distance_From_Plane
            durationScale = 2
        else:
            radius = np.real(cmath.sqrt(coord[0]**2 + coord[1]**2))
            heightCorrection = mapValue(radius,219,250,-8,0)
            pose[2,3] = drawing_distance_From_Plane + heightCorrection
            if(coord[2] == -1):
                durationScale = 2
            else:
                durationScale = 1

        q = getJointValues(pose, degrees=True)

        print_state(filename, [int(n) for n in coord])

        if(q[0,0] != -1):
            elbow_up_joints = q[0] + HOME
            elbow_up_joints = np.append(elbow_up_joints,closed_Gripper_Angle_Value)
            addPointToTrajectory(elbow_up_joints,durationScale)
            

    executeTrajectory()

    


def goToHome():
    q = [0,0,-90,0]
    setPhantomPose(q,0.5)

#Joint 1:99.61, Joint 2:-49.51, Joint 3:-111.62, Joint 4:71.78, Joint 5:0.59
def loadTool(load:bool):
    initializeTrajectory()
    trajectory_filename = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),"trajectories","load.txt")
    trajectory = getTrajectoryFromTextFile(trajectory_filename)
    durationScale = 1.5
    for i in range(0,2):
        coord = trajectory[i]
        pose = getPerpendicularOrientation(coord)
        pose[0,3] = coord[0]
        if(not load):
            pose[0,3] = coord[0] + 10 #Correction for unload
            pose[1,3] = coord[1] -0.5 #Correction for unload
        pose[1,3] = coord[1]    
        pose[2,3] = coord[2]
        q = getJointValues(pose,degrees=True)
        elbow_up_joints = q[0] + HOME
        if(load):
            elbow_up_joints = np.append(elbow_up_joints,open_Gripper_Angle_Value)
        else:
            elbow_up_joints = np.append(elbow_up_joints,closed_Gripper_Angle_Value)
        addPointToTrajectory(elbow_up_joints,durationScale)
    executeTrajectory()
    time.sleep(2)
    if(load):
        setGripperPosition(closed_Gripper_Dig_Value)
    else:
        setGripperPosition(open_Gripper_Dig_Value)
    time.sleep(1)
    initializeTrajectory()
    coord = trajectory[2]
    pose = getPerpendicularOrientation(coord)
    pose[0,3] = coord[0]
    pose[1,3] = coord[1]    
    pose[2,3] = coord[2]
    q = getJointValues(pose,degrees=True)
    elbow_up_joints = q[0] + HOME
    if(load):
        elbow_up_joints = np.append(elbow_up_joints,closed_Gripper_Angle_Value)
    else:
        elbow_up_joints = np.append(elbow_up_joints,open_Gripper_Angle_Value)
    addPointToTrajectory(elbow_up_joints,1)
    executeTrajectory()
    time.sleep(1)
    goToHome()


def parse_option(option, tool_loaded):

    if option == 9:
        exit()
    if option == 10:
        os.system('clear')
        print_menu()
        return tool_loaded

    if not tool_loaded:
        if option in range(1,6):
            print("Tool must be loaded to draw shapes")
            return tool_loaded
        if option == 6:
            print("No tool to unload")
            return tool_loaded
        if option == 7:
            tool_loaded = True
    else:
        if option==7:
            print("Tool already mounted")
            return tool_loaded

    start_time = time.time()

    if(tool_loaded):
        print("Tool Loaded")
    else:
        print("Tool Unloaded")

    if option == 1:
        draw(file_names[3])
        draw(file_names[4])
        goToHome()
    elif option == 2:
        draw(file_names[2])
        draw(file_names[7])
        goToHome()
    elif option == 3:
        draw(file_names[0])
        draw(file_names[5])
        draw(file_names[8])
        goToHome()
    elif option == 4:
        draw(file_names[6])
        goToHome()
    elif option == 5:
        draw(file_names[1])
        goToHome()
    elif option == 6:
        tool_loaded = False
        loadTool(False)
    elif option == 7:
        tool_loaded = True
        loadTool(True)
    elif option == 8:
        goToHome()
    
    end_time = time.time()
    print("Routine has ended.")
    print("Routine execution time:", int((end_time-start_time)*1000) , "ms")

    return tool_loaded

def print_menu():
    print("""Menu options:
                    (1) Draw Workspace 
                    (2) Draw Author's initials 
                    (3) Draw geometric shapes 
                    (4) Draw points 
                    (5) Draw free shape 
                    (6) Unload tool
                    (7) Load tool
                    (8) Go to home
                    (9) Exit
                    (10) Clear screen""")

def main():
    configMotors()
    tool_loaded = False
    print_menu()
    while True:
        try:
            option = int(input("Next action: "))
            if option not in range(1,11):
                raise ValueError
        except ValueError:
            os.system('clear')
            print_menu()
        else:
            tool_loaded = parse_option(option, tool_loaded)

if __name__ == "__main__":
    main()

