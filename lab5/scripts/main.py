import cmath
from math import atan2
from inv_kinematics import getJointValues
from parser import getTrajectoryFromTextFile
from jointCommand import *
import numpy as np
import time
import os 

file_names = ["circle.txt", "custom_part.txt", "d_letter.txt",
                "inner_ring.txt", "outter_ring.txt", "line123.txt",
                "point1to5.txt", "s_letter.txt", "triangle.txt"]

distance_From_Plane = 50

def print_state(filename, coord):
    print("Routine: {}. End effector: X: {} Y: {} Z: {} (mm)"
            .format(filename, coord[0], coord[1], coord[2]))

def draw(filename):
    trajectory_filename = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),"trajectories",filename)
    trajectory = getTrajectoryFromTextFile(trajectory_filename)

    for coord in trajectory:
        theta1 = atan2(coord[1],coord[0])
        cos_theta1 = np.real(cmath.cos(theta1))

        sin_theta1 = np.real(cmath.sin(theta1))
        pose = np.matrix([[cos_theta1,0,sin_theta1 ,0],
                          [sin_theta1,0,-cos_theta1,0],
                          [0         ,1,0          ,0],
                          [0        ,0 ,0          ,1]]) #Perpendicular to surface
        pose[0,3] = coord[0]
        pose[1,3] = coord[1]
        pose[2,3] = coord[2] + distance_From_Plane
        q = getJointValues(pose, degrees=True)
        elbow_up_joints = q[0]
        setPhantomPose(elbow_up_joints)
        time.sleep(0.001)

        print_state(filename, [int(n) for n in coord])

def parse_option(option, tool_loaded):
    if option == 9:
        exit()
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
    print("Tool state: Loaded")
    if option == 1:
        draw(file_names[3])
        draw(file_names[4])
    elif option == 2:
        draw(file_names[2])
        draw(file_names[7])
    elif option == 3:
        draw(file_names[0])
        draw(file_names[5])
        draw(file_names[8])
    elif option == 4:
        draw(file_names[6])
    elif option == 5:
        draw(file_names[1])
    elif option == 6:
        tool_loaded = False
        # TODO 
        print("Unload tool trajectory filename")
    elif option == 7:
        # TODO 
        print("Load tool trajectory filename")
    elif option == 8:
        # TODO
        print("Got-to-home trajectory filename")
    
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
            if option not in range(1,10):
                raise ValueError
        except ValueError:
            import os
            os.system('clear')
            print_menu()
        else:
            tool_loaded = parse_option(option, tool_loaded)

if __name__ == "__main__":
    main()

