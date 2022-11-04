import time
import numpy as np
import rospy
from dynamixel_workbench_msgs.srv import DynamixelCommand
from cmath import pi
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

#Publisher
pub = rospy.Publisher('/joint_trajectory', JointTrajectory, queue_size=0)
rospy.init_node('joint_publisher', anonymous=False)
#Trajectory
state = None
durationCount = 0
waitCount = 0
trajectory_point_duration = 0.15
wait_Per_Point = 0.8


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

def initializeTrajectory():
    global state
    state = JointTrajectory()
    state.header.stamp = rospy.Time.now()
    state.joint_names = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5"]
    global durationCount, waitCount
    durationCount = 0
    waitCount = 0

def addPointToTrajectory(angles, durationScale):
    global durationCount, waitCount
    durationCount = durationCount + trajectory_point_duration*durationScale
    point = JointTrajectoryPoint()
    point.positions =  np.multiply(angles,pi/180) 
    point.time_from_start = rospy.Duration(durationCount)
    state.points.append(point)
    waitCount = waitCount + wait_Per_Point*durationScale

def executeTrajectory():
    pub.publish(state)
    time.sleep(waitCount)

def configMotors():
    for i in range(1, 4):
        jointCommand("", i, "Torque_Limit", 900, 0)
        jointCommand("", i, "Torque_Enable", 1, 0)

def degrees2digital(degrees, min_deg=-150, max_deg=150):
    return list(
        map(lambda d: int((float(d) - min_deg) * 1024 / (max_deg - min_deg)), degrees)
    )


def setPhantomPose(q, waitTime):
    """Doesn't set gripper"""
    q = degrees2digital(q)
    for i in range(0, 4):
        jointCommand("", i+1, "Goal_Position", q[i],waitTime)
    



def setGripperPosition(servoValue: int = 0) -> None:
    """
    Set the gripper position to the specified 0 to 1023 value
    """
    jointCommand("", 5, "Goal_Position", servoValue, 0)
