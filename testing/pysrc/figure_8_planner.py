#!/usr/bin/env python
#
#

import rospy
import math

from sensor_msgs.msg import NavSatFix
from core_msgs.msg import *
from gps_helper import *

class Figure8Planner(object):

    def __init__(self):
        rospy.init_node('figure_8_planner')

        self.gpsHelper = GpsHelper()
        self.startLat = None
        self.startLon = None
        self.plan = None
        self.newPlan = False

        rospy.Subscriber('fix', NavSatFix, self.handleFix)
        self.planPub = rospy.Publisher('plan', Plan, latch=True)

    def doWork(self):
        atPoint = self.gpsHelper.atPoint(self.startLat, self.startLon)
        # it's no longer a "new plan" once we've moved away from our start point
        if not atPoint:
            self.newPlan = False
        elif not self.newPlan: # if we've come back around, reset the plan and go again
            rospy.loginfo("At the start, resetting the plan")
            # reset the plan
            self.resetPlan()
        
        if not self.plan and self.gpsHelper.isSet():
            rospy.loginfo("Publishing Figure-8 plan")
            self.startLat = self.gpsHelper.latitude
            self.startLon = self.gpsHelper.longitude
            self.plan = self.generateAndPublishPlan()
            self.newPlan = True

    def generateAndPublishPlan(self):

        carlen = 1.2192 # 4 ft in meters
        maxwheelangle = 40 * math.pi / 180
        carwidthhalf = 0.4572
        radius = carlen / math.tan(maxwheelangle) + carwidthhalf

        msg = Plan()
        # first semicircle of first circle
        p1 = PlanStep()
        p1.length = 2 * radius * maxwheelangle
        p1.angle  = maxwheelangle
        p1.radius = radius
        msg.steps.append(p1)
        # first semicircle of second circle
        p2 = PlanStep()
        p2.length = 2 * radius * (maxwheelangle)
        p2.angle  = (-maxwheelangle)
        p2.radius = radius
        msg.steps.append(p2)
        # second semicircle of second circle
        p3 = PlanStep()
        p3.length = 2 * radius * (maxwheelangle)
        p3.angle  = (-maxwheelangle)
        p3.radius = radius
        msg.steps.append(p3)
        # second semicircle of first circle
        p4 = PlanStep()
        p4.length = 2 * radius * (maxwheelangle)
        p4.angle  = (maxwheelangle)
        p4.radius = radius
        msg.steps.append(p4)
        
        self.planPub.publish(msg)
        return msg

    def resetPlan(self):
        self.plan = None

    def handleFix(self, data):
        self.gpsHelper.update(data.latitude, data.longitude, data.position_covariance)
   

def main():
    pl = Figure8Planner()
    r = rospy.Rate(1) # 1hz
    while not rospy.is_shutdown():
        pl.doWork()
        r.sleep()

if __name__ == "__main__":
    main()
