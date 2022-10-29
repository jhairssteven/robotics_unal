from jointCommand import jointCommand
from sendJointValues import sendJointValues
from setGripper import setGripper


motors_ids = [1 2 3 4 5]

#Movement parameters

drawing_Distance_From_Plane = 0; #cm
safe_Distance_From_Plane = 10; #cm


#Get Positions

# Motors configuration

jointCommand('', motors_ids[0], 'Torque_Limit', 300, 0)
jointCommand('', motors_ids[1], 'Torque_Limit', 500, 0)
jointCommand('', motors_ids[2], 'Torque_Limit', 300, 0)
jointCommand('', motors_ids[3], 'Torque_Limit', 300, 0)
jointCommand('', motors_ids[4], 'Torque_Limit', 300, 0)

jointCommand('', motors_ids[0], 'Torque_Enable', 1, 0)
jointCommand('', motors_ids[1], 'Torque_Enable', 1, 0)
jointCommand('', motors_ids[2], 'Torque_Enable', 1, 0)
jointCommand('', motors_ids[3], 'Torque_Enable', 1, 0)
jointCommand('', motors_ids[4], 'Torque_Enable', 1, 0)

# Initial gripper open position
setGripper(gripper_open_value)

# Movement
time.sleep(1)
for i in q_vector1:
    sendJointValues(i)    
setGripper(base_gripper_closed_value)
time.sleep(0.8)         # Stabnilization delay

for i in q_vector2:
    sendJointValues(i)
setGripper(gripper_open_value)
time.sleep(0.8)

for i in q_vector3:
    sendJointValues(i)
setGripper(load_gripper_closed_value)
time.sleep(0.8)

for i in q_vector4:
    sendJointValues(i)
setGripper(gripper_open_value)
time.sleep(0.5)