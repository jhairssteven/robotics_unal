import numpy as np
import rospy
from dynamixel_workbench_msgs.srv import DynamixelCommand
from cmath import pi
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint




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

def joint_publisher(angles):
    pub = rospy.Publisher('/joint_trajectory', JointTrajectory, queue_size=0)
    rospy.init_node('joint_publisher', anonymous=False)
    state = JointTrajectory()
    state.header.stamp = rospy.Time.now()
    state.joint_names = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5"]
    point = JointTrajectoryPoint()
    point.positions =  angles   
    point.time_from_start = rospy.Duration(0.5)
    state.points.append(point)
    pub.publish(state)
    rospy.sleep(3)

def configMotors(draw:bool):
    if(draw):
        for i in range(1, 4):
            jointCommand("", i, "Torque_Limit", 700, 0)
            jointCommand("", i, "Torque_Enable", 1, 0)
        jointCommand("", 4, "Torque_Limit", 900, 0)
        jointCommand("", 4, "Torque_Enable", 1, 0)
        jointCommand("", 5, "Torque_Limit", 1023, 0)
        jointCommand("", 5, "Torque_Enable", 1, 0)
    else:
        for i in range(1, 6):
            jointCommand("", i, "Torque_Limit", 600, 0)
            jointCommand("", i, "Torque_Enable", 1, 0)


def degrees2digital(degrees, min_deg=-150, max_deg=150):
    return list(
        map(lambda d: int((float(d) - min_deg) * 1024 / (max_deg - min_deg)), degrees)
    )


def setPhantomPose(q, sequentially:bool = False):
    # """Doesn't set gripper"""
    # q = degrees2digital(q)
    # waitTime = 0
    # if(sequentially):
    #     waitTime = 0.5
    # for i in range(0, 4):
    #     jointCommand("", i+1, "Goal_Position", q[i], waitTime)
    joint_publisher(np.multiply(q,(pi/180)))
    



def setGripperPosition(servoValue: int = 0) -> None:
    """
    Set the gripper position to the specified 0 to 1023 value
    """
    jointCommand("", 5, "Goal_Position", servoValue, 0)
