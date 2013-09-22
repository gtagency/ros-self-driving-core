#!/usr/bin/env python
"""Position control for a linear actuator implemented using PID control.

The algorithm is adapted from this example:
http://www.phidgets.com/docs/PID_Control_of_a_Linear_Actuator

and should work with any other node that accepts a LinearCommand
and publishes the current position of the linear actuator

"""

__author__ = 'Jesse Rosalia <jesse.rosalia@gatech.edu>'
__version__ = '1'

#import roslib; roslib.load_manifest('LinearPID')
from core_msgs.msg import LinearCommand
from core_srvs.srv import PidGains
from linear_pid.srv import TargetPosition
from std_msgs.msg import UInt32

import rospy
import math

#Motor controller parameters
accel = 100
maxOutput = 100
duration = 0 # run until we say stop

#Amount of error that can still be present when we stop
deadBand = 5

# component gains
Kp = 3
Ki = 2
Kd = 1

#
lastFeedback = 0
#Initial target position
targetPosition = 0

#Feedback update rate
dt = 0.008

#Accumulated terms
integral =0
derivative = 0

#Last output from algorithm
outputLast = 0
#Last error from algorithm
errorLast = 0

minFeedback=290
maxFeedback=940

canStopSpeed=15

#        //Variables
#        double error;                  // error is the difference between the target and the actual position
#        double errorlast;              // errorlast is the error in the previous iteration of the control loop
#        double integral;               // integral is the integral term of the control loop
#        double derivative;             // derivative is the derivative term of the control loop
#        double dt;                     // dt is the amount of time between iterations of the control loop
#        double maxOutput = 100;        // maxOutput is the selected maximum duty cycle of the motor controller
#        double deadBand = 0;           // deadBand is the amount of error that can still be present when the motor stops
#        bool started = false;          // started controls whether or not the control loop is active
#        double graphTimeCount = 0;     // graphTimeCount is the X position variable for the graph

lcPub = None

def setTargetPosition(req):
    global targetPosition,lastFeedback
    rng = maxFeedback - minFeedback
    if req.targetPosition < 0 or req.targetPosition > 1.0:
        rospy.logerr("TargetPosition must be between 0 and 1.0")
        return
    targetPosition = minFeedback + math.floor(req.targetPosition * rng)
    print "Setting target position to ", targetPosition 
    output = PID(lastFeedback)
    lc = LinearCommand()
    lc.speed = output
    lc.acceleration = accel
    lc.secondsDuration = duration
    lcPub.publish(lc)

# This function does the control system calculations and sets output to the duty cycle that the motor needs to run at.
#     feedback: input from the motor position sensor
def PID(feedback):
    global errorLast,targetPosition,deadBand,minFeedback,maxFeedback,integral,derivative,outputLast,dt,canStopSpeed
    # Calculate how far we are from the target
    error = targetPosition - feedback

    #TODO: we need a minimum speed, or a way of affecting 1 "step", otherwise we have the potential to sit at one position
    #forever (if say, the speed computed is too small to actually move the thing 
    # If the error is within the specified deadband, and the motor is moving slowly enough,
    # Or if the motor's target is a physical limit and that limit is hit (within deadband margins),
    if (math.fabs(error) <= deadBand and math.fabs(outputLast) < canStopSpeed) \
        or (targetPosition < (deadBand + 1 + minFeedback) and feedback < (deadBand + 1 + minFeedback)) \
        or (targetPosition > (maxFeedback - (1 + deadBand)) and feedback > (maxFeedback - (1 + deadBand))):
        # Stop the motor
        print "Stopping the motor"
        output = 0
        error = 0
        targetPosition = 0
    else:
        # Else, update motor duty cycle with the newest output value
        # This equation is a simple PID control loop
        output = ((Kp * error) + (Ki * integral)) + (Kd * derivative)
 
 
    # Prevent output value from exceeding maximum output specified by user, otherwise accumulate the integral
    if output >= maxOutput:
        output = maxOutput
    elif output <= -maxOutput:
        output = -maxOutput
    else:
        integral += (error * dt)
 
    derivative = (error - errorLast) / dt

    errorLast = error
    outputLast = output

    return output

def handle_position_feedback(data):
    global lastFeedback,targetPosition
    print data.data
    lastFeedback = data.data
    print lastFeedback
    if not targetPosition == 0:
        output = PID(data.data)
        print targetPosition, output, lastFeedback
        lc = LinearCommand()
        lc.speed = output
        lc.acceleration = accel
        lc.secondsDuration = duration
        lcPub.publish(lc)

def set_gains(req):
    global Kp, Ki, Kd
    print "Setting gains", req.proportional_gain, req.integral_gain, req.derivative_gain
    Ki = req.integral_gain
    Kp = req.proportional_gain
    Kd = req.derivative_gain

def main():
   
    global lcPub
 
    rospy.init_node("linear_pid")
    lcPub = rospy.Publisher('linear_command', LinearCommand)
    rospy.Subscriber('position', UInt32, handle_position_feedback)
    rospy.Service('target_position', TargetPosition, setTargetPosition)
    rospy.Service('set_pid_gains', PidGains, set_gains)

    rospy.spin()

if __name__ == "__main__":
    main()
