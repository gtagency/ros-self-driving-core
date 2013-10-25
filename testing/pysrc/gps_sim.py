#!/usr/bin/env python
#

import rospy
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistStamped

from testing.srv import *

from threading import Thread
from multiprocessing import Process
import time
import math
import signal
import sys

def signal_handler(signal, frame):
    print 'You pressed Ctrl+C!'
    sys.exit(0)

# NOTE: all times in seconds unless otherwise noted, all distance in meters, GPS in WGS-84 coordinates
class GpsSim(object):
    
    def __init__(self):
        rospy.init_node("gps_sim")
        self.fixPub = rospy.Publisher("fix", NavSatFix)
        rospy.Subscriber("vel_sim", TwistStamped, self.handleVelocity)
        rospy.Service("playControl", PlayControl, self.handlePlayControl)
        rospy.Service("setup", GpsSimSetup, self.handleGpsSimSetup)

        self.setupSimulator(rospy.get_param("~startLat", None),
                            rospy.get_param("~startLon", None),
                            rospy.get_param("~linear_velocity", None),
                            rospy.get_param("~angular_velocity", None),
                            rospy.get_param("~updateRateMS", None))

        self.orientation = 0 # is due east        
        self.lastTime = time.time()
        self.curTime  = self.lastTime

        self.playing = rospy.get_param("~startImmediately", None)
        rospy.loginfo("Start immediately: %s", self.playing)

    def setupSimulator(self, startLat, startLon, linear_velocity, angular_velocity, updateRateMS):

        self.startLat = startLat
        self.startLon = startLon
        self.curLat   = self.startLat
        self.curLon   = self.startLon
        self.linear_velocity = linear_velocity
        self.angular_velocity = angular_velocity
        self.updateRateMS = updateRateMS
        self.updateRateSecs = updateRateMS/1000.0 if updateRateMS else None

        self.testIfSetup()
        rospy.loginfo("Simulator parameters set:")
        if self.startLat and self.startLon:
            rospy.loginfo("start: %f, %f" % (self.startLat, self.startLon))
        if self.linear_velocity and self.angular_velocity:
            rospy.loginfo("velocity (linear and angular) %f, %f m/s" % (self.linear_velocity, self.angular_velocity))
        if self.updateRateSecs:
            rospy.loginfo("with %f second updates" % self.updateRateSecs)
        if self.isSetup:
            rospy.loginfo("Setup complete!") 

    def testIfSetup(self):
        self.isSetup = self.startLat != None and \
                       self.startLon != None and \
                       self.linear_velocity  != None and \
                       self.angular_velocity != None and \
                       self.updateRateSecs   != None

    def doWork(self):
        while True:
            # if no configuration set, just sleep for 1 second
            delay = self.updateRateSecs if self.updateRateSecs else 1.0
            time.sleep(delay)
            
            self.curTime = time.time()
            if self.playing:
                if self.isSetup:
                    elapsed = self.curTime - self.lastTime
                    self.lastTime = self.curTime
                    disp  = self.linear_velocity  * elapsed
                    self.orientation += self.angular_velocity * elapsed
                    if self.orientation > 2 * math.pi:
                        self.orientation -= 2 * math.pi
                    # because the size of one degree
                    self.curLat += (disp * math.sin(self.orientation) / self.calcLatDegreeLength(self.curLat, self.curLon))
                    self.curLon += (disp * math.cos(self.orientation) / self.calcLonDegreeLength(self.curLat, self.curLon))
                    self.publishCurLatLon()
                elif self.curLat and self.curLon:
                    # even if we're not set up, we need to publish our current position
                    # as that may bootstrap another part of the system
                    self.publishCurLatLon()

    def calcLatDegreeLength(self, lat, lon):
        m_per_mi = 1600
        return 69 * m_per_mi #A HACK

    def calcLonDegreeLength(self, lat, lon):
        m_per_mi = 1600
        return 69 * m_per_mi # A SUPER HACK

    def publishCurLatLon(self):
        msg = NavSatFix()
        msg.header.stamp = rospy.Time(self.curTime)
        msg.latitude = self.curLat
        msg.longitude = self.curLon
        # very small error for now
        msg.position_covariance = [0]*9
        msg.position_covariance[0] = 5**2
        msg.position_covariance[3] = 5**2
        rospy.loginfo("Publishing position %f, %f at time %f", self.curLat, self.curLon, self.curTime)
 
        self.fixPub.publish(msg)
    
    def handlePlayControl(self, req):
        if self.isSetup:
            self.playing = not self.playing
            rospy.loginfo("Received command to %s" % ("play" if self.playing else "stop"))
            # update the last time, so we can accurately compute displacement
            # when we start up
            self.lastTime = time.time()
        else:
            rospy.logerr("Unable to control play.  Simulator not setup.")

    def handleGpsSimSetup(self, req):
        self.setupSimulator(req.startLat, req.startLon, req.linear_velocity, req.angular_velocity, req.updateRateMS)

    def handleVelocity(self, data):
        # NOTE: only use linear.y, angular.z velocity
        self.linear_velocity = data.twist.linear.y 
        self.angular_velocity = data.twist.angular.z
        self.testIfSetup()
        rospy.loginfo("Velocity message received. linear: %f, angular: %f, setup: %s" % (self.linear_velocity, self.angular_velocity, self.isSetup))

def main():
    signal.signal(signal.SIGINT, signal_handler)
    print 'Press Ctrl+C'

    g = GpsSim()
    g.doWork()

if __name__ == "__main__":
    main()
