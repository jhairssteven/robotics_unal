from jointCommand import jointCommand

def setGripperPosition(servoValue:int=0)->None:
    """
    Set the gripper position to the specified 0 to 1023 value
    """
    jointCommand('', motors_ids[4], 'Goal_Position', servoValue, 0)