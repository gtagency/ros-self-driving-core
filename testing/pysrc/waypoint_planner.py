#!/usr/bin/env python
#

import rospy
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Point
from testing.srv import *
from core_msgs.msg import *
from gps_helper import *

import time
import math

# NOTE: all times in seconds unless otherwise noted, all distance in meters, GPS in WGS-84 coordinates
class WaypointPlanner(object):
    
    def __init__(self):
        rospy.init_node("waypoint_planner")

        self.gpsHelper = GpsHelper()
        self.destLat = None
        self.destLon = None 
        rospy.Subscriber("moveTowards", Point, self.handleMoveTowards)
        rospy.Subscriber('fix', NavSatFix, self.handleFix)
        self.planPub = rospy.Publisher('plan', Plan, latch=True)

    def generateAndPublishPlan(self, distance, bearing):
        carlen = 1.2192 # 4 ft in meters
        maxwheelangle = 40 * math.pi / 180
        carwidthhalf = 0.4572
        radius = carlen / math.tan(maxwheelangle) + carwidthhalf

        print "Bearing: ", bearing, "Radius: ", radius
        # curvedist = the arc length to reach the bearing - one radius length, so we can correct our wheels
        curvedist = radius * bearing - radius
        msg = Plan()
        chordlens = 0
        if curvedist > 0:
            # circle to get perpendicular to the bearing (overshoot)
            p1 = PlanStep()
            p1.length = curvedist
            p1.angle  = bearing
            p1.radius = radius
            msg.steps.append(p1)
            # 90 degrees in the other direction, to get on the bearing
            p2 = PlanStep()
            p2.length = radius * math.pi / 2
            p2.angle  = -(math.pi / 2)
            p2.radius = radius
            msg.steps.append(p2)
            # distance covered
            chordlens = radius * 2 * (math.sin(bearing/2) + math.sin(math.pi/2))
        else: 
            rospy.logwarn("Initial curve distance is negative.  That's not right.")
        # straight line to goal
        p3 = PlanStep()
        p3.length = distance - chordlens
        p3.angle  = 0
        p3.radius = 0
        msg.steps.append(p3)
        
        self.planPub.publish(msg)
        return msg

    def handleFix(self, data):
        self.gpsHelper.update(data.latitude, data.longitude, data.position_covariance)
        if self.destLat != None and self.destLon != None:
            distandbrng = self.gpsHelper.distanceAndBearingTo(self.destLat, self.destLon)
            self.generateAndPublishPlan(*distandbrng)
            # reset the destinations so we don't replan constantly
            self.destLat = None
            self.destLon = None

    def handleMoveTowards(self, data):
        # YES I KNOW x and y are wrong here...x = lat, y = lon
        self.destLat = data.x
        self.destLon = data.y
        if self.gpsHelper.isSet():
            distandbrng = self.gpsHelper.distanceAndBearingTo(self.destLat, self.destLon)
            self.generateAndPublishPlan(*distandbrng)
            # reset the destinations so we don't replan constantly
            self.destLat = None
            self.destLon = None

def main():
    pl = WaypointPlanner()
    rospy.spin()

if __name__ == "__main__":
    main()
