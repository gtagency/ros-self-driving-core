#!/usr/bin/env python
#

import rospy
from sensor_msgs.msg import NavSatFix
from testing.srv import *

from threading import Thread
from multiprocessing import Process
import time

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
        rospy.Service("playControl", PlayControl, self.handlePlayControl)
        rospy.Service("setup", GpsSimSetup, self.handleGpsSimSetup)

        self.startLat = None
        self.startLon = None
        self.endLat   = None
        self.endLon   = None
        self.curLat   = None
        self.curLon   = None
        self.curTime  = None
        self.velocity = [0, 0] 
        self.updateRateSecs = None
        self.lastTime = None
        
        self.playing = False
        self.isSetup = False

#//        t = Thread(target=self.doWork)
 # //      t.start()

    def doWork(self):
        while True:
            rospy.logdebug("looping")
            # if no configuration set, just sleep for 1 second
            delay = self.updateRateSecs if self.updateRateSecs else 1.0
            time.sleep(delay)
            
            self.curTime = time.time()
            if self.playing:
                rospy.logdebug("Here we go")
                elapsed = self.curTime - self.lastTime
                self.lastTime = self.curTime
                displacement = [v * elapsed for v in self.velocity]
                # because the size of one degree
                self.curLat += (displacement[0] / self.calcLatDegreeLength(self.curLat, self.curLon))
                self.curLon += (displacement[1] / self.calcLonDegreeLength(self.curLat, self.curLon))
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
        msg.position_covariance[0] = 1
        msg.position_covariance[3] = 1
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
            rospy.logerror("Unable to control play.  Simulator not setup.")

    def handleGpsSimSetup(self, req):
        self.startLat = req.startLat
        self.startLon = req.startLon
        self.curLat   = req.startLat
        self.curLon   = req.startLon
        self.endLat   = req.endLat
        self.endLon   = req.endLon
        
        self.velocity = req.velocity # 2-vector, meters per second
        self.updateRateSecs = req.updateRateMS/1000.0
        self.isSetup = True
        rospy.loginfo("Setup complete.  Simulating navigation from %f, %f to %f, %f at x, y velocities %d, %d with %f second updates"
                    % (self.startLat, self.startLon, self.endLat, self.endLon, self.velocity[0], self.velocity[1], self.updateRateSecs))

def main():
    signal.signal(signal.SIGINT, signal_handler)
    print 'Press Ctrl+C'

    g = GpsSim()
    g.doWork()

if __name__ == "__main__":
    main()
