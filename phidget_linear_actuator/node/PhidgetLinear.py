#!/usr/bin/env python
"""Linear Actuator control using Phidgets 1065 motor controller and
Feedback-enabled linear actuator.

Based on the Phidgets HC Motor Control ROS service for CoroBot

"""

__author__ = 'Jesse Rosalia <jesse.rosalia@gatech.edu>'
__version__ = '1'

import roslib; roslib.load_manifest('phidget_linear_actuator')
from phidget_linear_actuator.srv import *
from core_msgs.msg import *
from std_msgs.msg import UInt32

import rospy
from threading import Timer
from ctypes import *

from Phidgets.Devices.MotorControl import MotorControl
from Phidgets.PhidgetException import PhidgetErrorCodes, PhidgetException
from Phidgets.Events.Events import EncoderPositionUpdateEventArgs

motorControl = 0
invert_speed = True
linear = 0
feedbackSensor = 0
minAcceleration = 0
maxAcceleration = 0
minSpeed = -100
maxSpeed = 100
timer = 0
posdataPub = None
position = 0

def stop():
    print "Stopping at", position
    try:
        motorControl.setVelocity(linear,0);
    except PhidgetException as e:
        rospy.logerr("Failed in setVelocity() %i: %s", e.code, e.details)
        return(False)
    return(True)

def bounded_value(val, minVal, maxVal):
    bv = val
    if bv > maxVal:
        bv = maxVal
    elif bv < minVal:
        bv = minVal
   
    return bv 

def move(request):
    """Cause the linear actuator to move or stop moving

    Request a common acceleration, wheel directions and wheel speeds

    """
    global timer, invert_speed
    if timer:
        timer.cancel();
        rospy.logdebug(
            "Speed: %d, Acceleration: %d", 
            request.speed,
            request.acceleration
            )

    acceleration = bounded_value(request.acceleration, float(minAcceleration), float(maxAcceleration))
    # NOTE: negate the speed so + is out and - is back
    # FIXME: this should be an attribute
    speed        = bounded_value(request.speed,        float(minSpeed), float(maxSpeed))
    if invert_speed:
        speed = -speed

    rospy.logdebug(
            "Speed: %d, Acceleration: %d", 
            speed,
            acceleration
            )

    try:
        motorControl.setAcceleration(linear, acceleration);
		
    except PhidgetException as e:
        rospy.logerr("Failed in setAcceleration() %i: %s", e.code, e.details)
        return(False)

    try:
        motorControl.setVelocity(linear, speed);

    except PhidgetException as e:
        rospy.logerr("Failed in setVelocity() %i: %s", e.code, e.details)
        return(False)

    if request.secondsDuration != 0:
        timer = Timer(request.secondsDuration, stop)
        timer.start()
    return(True)

def mcAttached(e):
    return

def mcDetached(e):
    return

def mcError(e):
    return

def mcCurrentChanged(e):
    return

def mcInputChanged(e):
    return

def mcVelocityChanged(e):
    return

def mcSensorUpdated(e):
    #print e.index, e.value
    # Only publish sensor updates on the feedback sensor
    if e.index == feedbackSensor:
        position = e.value

        if posdataPub:
            posdataPub.publish(position)
        #msg.header.stamp = rospy.Time.now()
    return

def leftEncoderUpdated(e):

    return

def rightEncoderUpdated(e):

    return

def setupMoveService():
    """Initialize the PhidgetLinear service

    Establish a connection with the Phidget 1065 Motor Control and
    then with the ROS Master as the service PhidgetLinear

    """

    rospy.init_node(
            'PhidgetLinear',
            log_level = rospy.DEBUG
            )
    
    serial_no = rospy.get_param("~serial_no", 0)
    if not serial_no == 0:
        rospy.loginfo("Using motor controller with serial number %d", serial_no)
    else:
        rospy.loginfo("No serial number specified.  This is fine for systems with one controller, but may behave unpredictably for systems with multiple motor controllers")

    global motorControl, minAcceleration, maxAcceleration, timer, invert_speed, posdataPub
    timer = 0
    try:
        motorControl = MotorControl()
    except:
        rospy.logerr("Unable to connect to Phidget Motor Control")
     	return

    try:
        motorControl.setOnAttachHandler(mcAttached)
        motorControl.setOnDetachHandler(mcDetached)
        motorControl.setOnErrorhandler(mcError)
        motorControl.setOnCurrentChangeHandler(mcCurrentChanged)
        motorControl.setOnInputChangeHandler(mcInputChanged)
        motorControl.setOnVelocityChangeHandler(mcVelocityChanged)
        motorControl.setOnSensorUpdateHandler(mcSensorUpdated)
        if serial_no != 0:
            motorControl.openPhidget(serial_no)
        else:
            motorControl.openPhidget()

        #attach the board
        motorControl.waitForAttach(10000)
    except PhidgetException as e:
        rospy.logerr("Unable to register the handlers: %i: %s", e.code, e.details)
        return
    except AttributeError as e:
        rospy.logerr("Unable to register the handlers: %s", e)
        return
    except:
        rospy.logerr("Unable to register the handlers: %s", sys.exc_info()[0])
        return
     

    if motorControl.isAttached():
        rospy.loginfo(
                "Device: %s, Serial: %d, Version: %d",
                motorControl.getDeviceName(),
                motorControl.getSerialNum(),
                motorControl.getDeviceVersion()
                )

    minAcceleration = motorControl.getAccelerationMin(linear)
    maxAcceleration = motorControl.getAccelerationMax(linear)

    invert_speed = rospy.get_param('~invert_speed', False)

    phidgetMotorTopic = rospy.Subscriber("PhidgetLinear", LinearCommand ,move)
    phidgetMotorService = rospy.Service('PhidgetLinear',Move, move)
    posdataPub = rospy.Publisher("position", UInt32, latch=True)
    rospy.spin()

if __name__ == "__main__":
    setupMoveService()

    try:
        motorControl.closePhidget()
    except:
        pass
