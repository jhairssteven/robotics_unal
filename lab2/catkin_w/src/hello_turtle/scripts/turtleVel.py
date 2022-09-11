#!/usr/bin/env python3
import rospy
import argparse
from geometry_msgs.msg import Twist

class VelPub():
    def __init__(self):
        self.pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        rospy.init_node('velPub', anonymous=False)
        self.vel = Twist()
        self.rate = rospy.Rate(10) #0.1s
    
    def pubVel(self, vx, az):
        '''Move in X direction at `vx` and rotate in Z `az` radians'''
        self.vel.linear.x = vx
        self.vel.angular.z = az
        rospy.loginfo(self.vel)
        self.pub.publish(self.vel)
        self.rate.sleep()

def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('-vx', type = float, default = 2)
    parser.add_argument('-ang', type = float, default = 1)
    return parser.parse_args()

if __name__ == "__main__":
    args = parse_args()
    velPub = VelPub()
    while(1):
        try:
            velPub.pubVel(args.vx, args.ang)
        except rospy.ROSInterruptException:
            pass
        
