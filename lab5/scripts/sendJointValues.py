import numpy as np
from jointCommand import jointCommand

def sendJointValues(q:np.array = np.array([0, 0, 0, 0]) )->None:
    """
    Set all the joints of the robot to the position specified by the input array
    """
    q_raw=deg2raw(np.degrees(q))
    print(q_raw)
    jointCommand('', motors_ids[0], 'Goal_Position', q_raw[0], 0)
    jointCommand('', motors_ids[1], 'Goal_Position', q_raw[1], 0)
    jointCommand('', motors_ids[2], 'Goal_Position', q_raw[2], 0)
    jointCommand('', motors_ids[3], 'Goal_Position', q_raw[3], 0)

def deg2raw(input_list: list = [0,0,0,0], min_deg: int = -150, max_deg: int = 150)->list:
"""
Convert degrees array to 10 bit motor control value.
"""
out_list = [0,0,0,0]
for i in range(len(input_list)):
    out_list[i] = int( ((input_list[i] - min_deg)*1024)/(max_deg-min_deg) )
return out_list
