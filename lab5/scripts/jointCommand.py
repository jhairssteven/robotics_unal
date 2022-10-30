import rospy
from dynamixel_workbench_msgs.srv import DynamixelCommand


def jointCommand(command, motor_id, addr_name, value, time):
    rospy.wait_for_service("dynamixel_workbench/dynamixel_command")
    try:
        dynamixel_command = rospy.ServiceProxy(
            "/dynamixel_workbench/dynamixel_command", DynamixelCommand
        )
        result = dynamixel_command(command, motor_id, addr_name, value)
        rospy.sleep(time)
        return result.comm_result
    except rospy.ServiceException as exc:
        print(str(exc))


def configMotors():
    for i in range(0, 5):
        jointCommand("", i, "Torque_Limit", 300, 0)
        jointCommand("", i, "Torque_Enable", 1, 0)


def degrees2digital(degrees, min_deg=-150, max_deg=150):
    return list(
        map(lambda d: (float(d) - min_deg) * 1024 / (max_deg - min_deg), degrees)
    )


def setPhantomPose(q):
    """Doesn't set gripper"""
    q = degrees2digital(q)
    for i in range(0, 4):
        jointCommand("", i, "Goal_Position", q[i], 0)

def setGripperPosition(servoValue: int = 0) -> None:
    """
    Set the gripper position to the specified 0 to 1023 value
    """
    jointCommand("", 4, "Goal_Position", servoValue, 0)
