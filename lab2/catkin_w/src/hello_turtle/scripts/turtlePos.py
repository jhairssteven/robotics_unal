#!/usr/bin/env python3
import rospy
import argparse
from turtlesim.srv import TeleportAbsolute, TeleportRelative

''' Move to the specified position if m=1 (absolute movement)
    Otherwise, if m=0, move relative to the current position and orientation.'''
def turtleMov(x, y, a, m=1):
    service = '/turtle1/teleport_' + ('absolute' if m else 'relative')
    mode = TeleportAbsolute if m else TeleportRelative
    rospy.wait_for_service(service)
    try:
        tp = rospy.ServiceProxy(service, mode)
        tp(x, y, a) if m else tp(x, a)
        print('x: {}, y: {}, ang: {}, m: {}'.format(x, y, a, 'a' if m else 'r'))
    except rospy.ServiceException as e:
        print(str(e))

def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('-x', type = float, default = 5.0)
    parser.add_argument('-y', type = float, default = 5.0)
    parser.add_argument('-ang', type = float, default = 0)
    parser.add_argument('-m', type = float, default = 1)
    args = parser.parse_args()
    return args

if __name__ == "__main__":
    args = parse_args()
    turtleMov(args.x, args.y, args.ang, args.m)
