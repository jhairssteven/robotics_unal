#!/usr/bin/env python3

from pynput.keyboard import Key, KeyCode
from std_srvs.srv import Empty
from numpy import pi
import rospy

from turtleVel import VelPub
from turtlePos import turtleMov
from KeyListener import KeyListener

def key_turtle_control():
    kb = KeyListener() # Start listening for key presses
    turtleMov(5, 5, 0, 1) # Mov to screen's center
    velPub = VelPub()
    while not rospy.is_shutdown():
        if KeyCode.from_char('w') == kb.lastKey:
            velPub.pubVel(1, 0)
        if KeyCode.from_char('s') == kb.lastKey:
            velPub.pubVel(-1, 0)
        if KeyCode.from_char('a') == kb.lastKey:
            velPub.pubVel(0, pi/2)
        if KeyCode.from_char('d') == kb.lastKey:
            velPub.pubVel(0, -pi/2)
        if KeyCode.from_char('r') == kb.lastKey:
            turtleMov(5, 5, 0, 1)
            rospy.wait_for_service('/clear')  # Clear the trajectory
            clearTrajectory = rospy.ServiceProxy('/clear', Empty)
            clearTrajectory()
        if Key.space == kb.lastKey:
            turtleMov(0, 0, pi, 0)
        if Key.esc == kb.lastKey:
            break

if __name__ == '__main__':
    try:
        key_turtle_control()
    except rospy.ROSInterruptException:
        pass