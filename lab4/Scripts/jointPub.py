from cmath import pi
from turtle import pos
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

HOME = [0,0,-87,0,0] #Our HOME position in absolute coordinates
pose1 = [0,0,0,0,0]
pose2 = [-20,20,-20,20,0] 
pose3 = [-30,30,-30,30,0] 
pose4 = [-90,15,-55,17,0] 
pose5 = [-90,45,-55,45,10]

positions = [pose1,pose2,pose3,pose4,pose5]

def joint_publisher(input_pose):
    pub = rospy.Publisher('/joint_trajectory', JointTrajectory, queue_size=0)
    rospy.init_node('joint_publisher', anonymous=False)
    
    while not rospy.is_shutdown():
        state = JointTrajectory()
        state.header.stamp = rospy.Time.now()
        state.joint_names = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5"]
        point = JointTrajectoryPoint()
        point.positions =  input_pose   
        point.time_from_start = rospy.Duration(0.1)
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