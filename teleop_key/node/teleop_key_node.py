#!/usr/bin/env python
"""Keyboard teleop for any system that uses the phidget_linear_actuator and PhidgetMotor nodes for control.
"""

__author__ = 'Jesse Rosalia <jesse.rosalia@gatech.edu>'
__version__ = '1'

import roslib; roslib.load_manifest('teleop_key')
from core_msgs.msg import *
from core_srvs.srv import Move
from std_msgs.msg import Float32

import curses
from curses_keyboard import CursesKeyboard

import rospy
import math
import sys

steerPub = None
steerPctPub = None
motorPub = None
fast = False
steerReturn = False

def steer(direction):
    """
        Steering in one direction.  This may also stop steering
        if direction == 0
    """
    global steerPub

    lc = LinearCommand()
    lc.speed = 100 * direction
    lc.secondsDuration = 0
    lc.acceleration = 100
    steerPub.publish(lc)

def steerPct(pct):
    """
        Steering to a specific location by % of the whole steering range.  E.g.
        the center is 50% (0.5)
    """
    global steerPctPub

    steerPctPub.publish(pct)

def drive(direction):
    """
        Driving in one direction.  This may also stop the car
        if direction == 0
    """
    global motorPub, fast

    speed = 100 if fast else 50
    print speed
    mc = MotorCommand()
    mc.leftSpeed = speed * direction
    mc.rightSpeed = speed * direction
    mc.secondsDuration = 0 # until we say stop
    mc.acceleration = 100
    motorPub.publish(mc)


def fullStop():
    """ Stop as soon as possible
    """
    drive(0)
    steer(0)

def keyDownHandler(key):
    global fast, steerReturn
    if key == curses.KEY_LEFT or key == curses.KEY_RIGHT:
        steer(-1 if key == curses.KEY_LEFT else 1)

    if key == curses.KEY_DOWN or key == curses.KEY_UP:
        drive(-1 if key == curses.KEY_DOWN else 1)
    
    if key == ord('s'):
        print "STOP PRESSED"
        fullStop()
    
    if key == ord('f'):
        fast = not fast
        if fast:
            print "Fast speed enabled"
        else:
            print "Fast speed disabled"

    if key == ord('r'):
        steerReturn = not steerReturn
        if steerReturn:
            print "Steering return enabled"
        else:
            print "Steering return disabled"

    if key == ord('x'):
        print "Exiting"
        sys.exit(0)

def keyUpHandler(key):
    global steerReturn
    if key == curses.KEY_LEFT or key == curses.KEY_RIGHT:
        if steerReturn:
            steerPct(0.5) # return to center
        else:
            steer(0)

    if key == curses.KEY_DOWN or key == curses.KEY_UP:
        drive(0)

def main():
    global motorPub, steerPub, steerPctPub

    rospy.init_node('teleop_key')
    motorPub = rospy.Publisher('PhidgetMotor', MotorCommand)
    steerPub = rospy.Publisher('Steering', LinearCommand)
    steerPctPub = rospy.Publisher('SteeringPosition', Float32)
    
    print 'exit with x'
    c = CursesKeyboard(keyDownHandler, keyUpHandler)
    c.read()

if __name__ == "__main__":
    main()
