from cmath import pi
from turtle import pos
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

HOME = [0,0,-87,0,-58.30]
pos1 = [0,0,0,0,0]
pos2 = [-20,20,-20,20,0] 
pos3 = [-30,30,-30,30,0] 
pos4 = [-90,15,-55,17,0] 
pos5 = [-90,45,-55,45,10]

positions = [pos1,pos2,pos3,pos4,pos5]

def joint_publisher(input_positions):
    pub = rospy.Publisher('/joint_trajectory', JointTrajectory, queue_size=0)
    rospy.init_node('joint_publisher', anonymous=False)
    
    while not rospy.is_shutdown():
        state = JointTrajectory()
        state.header.stamp = rospy.Time.now()
        state.joint_names = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5"]
        point = JointTrajectoryPoint()
        point.positions =  input_positions   
        point.time_from_start = rospy.Duration(0.5)
        state.points.append(point)
        pub.publish(state)
        print('published command')
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        inputvalue = int(input("Enter desired position (0 for HOME):"))
        posRad = positions[inputvalue]
        for i in range(len(posRad)):
            posRad[i] = (posRad[i]+ HOME[i]) * (pi/180) 
        joint_publisher(posRad)
    except rospy.ROSInterruptException:
        pass