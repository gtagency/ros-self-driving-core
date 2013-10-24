#!/usr/bin/env python
#
#

import rospy
import time
import math

from core_msgs.msg import *
from geometry_msgs.msg import *
#NOTE: this ONLY converts motor commands and linear commands to Twist velocity, and assumes we're only ever going
# linearly in the direction of the car, and angularly around the Z axis (yaw).  As such, we do not need to compute
# any positions or trajectories

#NOTE: doesnt handle acceleration

class Motion(object):
    def __init__(self, rate, duration, maxInRate, maxOutRate):
        self.maxInRate = maxInRate
        self.maxOutRate = maxOutRate
        self.rate = rate
        self.duration = duration
        self.last_time = time.time()

    def isDone(self):
        return self.duration <= 0

    def step(self):
        cur_time = time.time()
        elapsed = min(cur_time - self.last_time, self.duration)
        self.duration -= elapsed
        self.last_time = cur_time
        return self.rate * self.maxOutRate / self.maxInRate
        
#class Locomotion(object):


#    def __init__(self, rate, duration):
#        self.rate = rate
#        self.duration = duration
#        self.last_time = time.time()

#    def step(self, delta_t):
#        cur_time = time.time()
#        elapsed = min(cur_time - self.last_time, self.duration)
#        self.duration -= elapsed
#        self.last_time = cur_time
#        return self.rate * maxRate / maxIncomingRate

#class SteeringMotion(object):
#    def __init__(self, rate, duration):
#        self.rate = rate
#        self.duration = duration
#        self.last_time = time.time()
        
#    def step(self, delta_t, vel_vector):
#        cur_time = time.time()
#        elapsed = min(cur_time - self.last_time, self.duration)
#        self.duration -= elapsed
#        self.last_time = cur_time
#        return (self.maxLinearRate * self.rate/maxIncomingRate) * steeringRes

class ControlSim(object):

    def __init__(self):
        rospy.init_node('control_sim')

        self.maxVelocity = rospy.get_param('~maxVelocity', 1)

        rospy.Subscriber('PhidgetLinear', LinearCommand, self.handleLinearCommand)
        rospy.Subscriber('PhidgetMotor',  MotorCommand,  self.handleMotorCommand)
        self.velPub = rospy.Publisher('vel', TwistStamped)

        # these are set when ros messages are recieved
        self.locomotion = None
        self.steering = None

    def step(self):
        print "Steppin"
        linVel = None
        if self.locomotion:
            linVel = self.locomotion.step()
            if self.locomotion.isDone():
                self.locomotion = None

        steerVel = None
        if self.steering:
            steerVel = self.steering.step()
            if not linVel:
                steerVel = None
            if self.steering.isDone():
                self.steering = None

        if linVel or steerVel:
            msg = TwistStamped()
            msg.header.stamp = rospy.Time.now()
            # only set y and z...we only have velocity in the direction of the car, and we're
            # only affecting yaw with our turning
            # TODO: 
            msg.twist.linear.y = linVel if linVel else 0.0
            msg.twist.angular.z = steerVel if steerVel else 0.0
            self.velPub.publish(msg)

    def handleLinearCommand(self, data):
        # from LA spec sheet
        maxLinearRate = 1.7  # "/s
        # estimation of angle from center to lock (on car)
        steeringAngle = 40 * math.pi / 180.0   # radian
        # linear distance from lock to lock
        linearDistance = 4.0 # "
        
        steeringRes = (2 * steeringAngle) / linearDistance # radians/" traveled
        maxIncomingRate = 100
        # TODO: blend the previous command with this command
        self.steering = Motion(data.speed, data.secondsDuration, maxIncomingRate, maxLinearRate * steeringRes)

    def handleMotorCommand(self, data):
        maxForwardRate = 2.35 #m/s equiv of ~5mph
        maxIncomingRate = 100
        # TODO: blend the previous command with this command
        self.locomotion = Motion(data.leftSpeed, data.secondsDuration, maxIncomingRate, maxForwardRate)


def main():

    cs = ControlSim()
        
    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        cs.step()
        r.sleep()

if __name__ == "__main__":
    main()
