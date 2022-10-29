from inv_kinematics import *
from jointCommand import *
from parser import *

configMotors()

pose = np.matrix(""" 0.7544   -0.1330    0.6428  188.5789;
                        0.6330   -0.1116   -0.7660  158.2365;
                        0.1736    0.9848    0.0000  234.1759;
                        0         0         0    1.0000""")

trajectory_filename = "..\\trajectories\\circle.txt"
trajectory = getTrajectoryFromTextFile(trajectory_filename)

for point in trajectory:
    pose[0,3] = point[0]
    pose[1,3] = point[1]
    pose[2,3] = point[2]
    q = getJointValues(pose, degrees=True)
    setPose(q[0])
