"""
Allows to use the service dynamixel_command 
"""
import rospy
import time
# from std_msgs.msg import String
from dynamixel_workbench_msgs.srv import DynamixelCommand

__author__ = "F Gonzalez, S Realpe, JM Fajardo"
__credits__ = ["Felipe Gonzalez", "Sebastian Realpe", "Jose Manuel Fajardo", "Robotis"]
__email__ = "fegonzalezro@unal.edu.co"
__status__ = "Test"

def jointCommand(command, id_num, addr_name, value, time):
    #rospy.init_node('joint_node', anonymous=False)
    rospy.wait_for_service('dynamixel_workbench/dynamixel_command')
    try:        
        dynamixel_command = rospy.ServiceProxy(
            '/dynamixel_workbench/dynamixel_command', DynamixelCommand)
        result = dynamixel_command(command,id_num,addr_name,value)
        rospy.sleep(time)
        return result.comm_result
    except rospy.ServiceException as exc:
        print(str(exc))

if __name__ == '__main__':
    try:
        # Goal_Position (0,1023)
        # Torque_Limit (0,1023)   
        jointCommand('', 1, 'Torque_Limit', 600, 0)
        jointCommand('', 2, 'Torque_Limit', 500, 0)
        jointCommand('', 3, 'Torque_Limit', 400, 0)
        jointCommand('', 4, 'Torque_Limit', 400, 0)
        while(True):
            inputvalue = int(input("Enter desired position (0 for HOME):"))
            print(inputvalue)
            if(inputvalue == 0):
                jointCommand('', 1, 'Goal_Position', 512, 0.5)
                jointCommand('', 2, 'Goal_Position', 512, 0.5)
                jointCommand('', 3, 'Goal_Position', 218, 0.5)
                jointCommand('', 4, 'Goal_Position', 512, 0.5)
                jointCommand('', 5, 'Goal_Position', 313, 0.5)
            elif(inputvalue == 1):
                jointCommand('', 1, 'Goal_Position', 444, 0.5)
                jointCommand('', 2, 'Goal_Position', 584, 0.5)
                jointCommand('', 3, 'Goal_Position', 150, 0.5)
                jointCommand('', 4, 'Goal_Position', 585, 0.5)
                jointCommand('', 5, 'Goal_Position', 313, 0.5)
            elif(inputvalue == 2):
                jointCommand('', 1, 'Goal_Position', 616, 0.5)
                jointCommand('', 2, 'Goal_Position', 415, 0.5)
                jointCommand('', 3, 'Goal_Position', 323, 0.5)
                jointCommand('', 4, 'Goal_Position', 412, 0.5)
                jointCommand('', 5, 'Goal_Position', 313, 0.5)
            elif(inputvalue == 3):
                jointCommand('', 1, 'Goal_Position', 205, 0.5)
                jointCommand('', 2, 'Goal_Position', 566, 0.5)
                jointCommand('', 3, 'Goal_Position', 30, 0.5)
                jointCommand('', 4, 'Goal_Position', 579, 0.5)
                jointCommand('', 5, 'Goal_Position', 313, 0.5)
            elif(inputvalue == 4):
                jointCommand('', 1, 'Goal_Position', 205, 0.5)
                jointCommand('', 2, 'Goal_Position', 668, 0.5)
                jointCommand('', 3, 'Goal_Position', 30, 0.5)
                jointCommand('', 4, 'Goal_Position', 668, 0.5)
                jointCommand('', 5, 'Goal_Position', 343, 0.5)
    except rospy.ROSInterruptException:
        pass