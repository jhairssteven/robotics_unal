#!/usr/bin/env python
from cmath import pi
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import JointState

HOME = [0,0,-87,0,-58.30]

def callback(data):
    result = "";
    for i in range(5):
        position = data.position[i] * (180/(pi)) - HOME[i]
        result += "Joint "+str((i+1))+":"+ "{:.2f}".format(position)
        if(i<4):
            result +=", "
    print(result)
    
def listener():
    rospy.init_node('joint_listener', anonymous=True)
    rospy.Subscriber("/dynamixel_workbench/joint_states", JointState, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        while not rospy.is_shutdown():
            listener()
    except rospy.ROSInterruptException:
        pass