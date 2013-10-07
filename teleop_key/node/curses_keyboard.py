#!/usr/bin/env python
"""Keyboard teleop for any system that uses the phidget_linear_actuator and PhidgetMotor nodes for control.
"""

__author__ = 'Jesse Rosalia <jesse.rosalia@gatech.edu>'
__version__ = '1'

import math
import sys
import curses

from time import time

class TrackedKey(object):
    def __init__(self, keyCode):
        self.keyCode = keyCode
        self.firstTime = True

    def isKey(self, keyCode):
        return self.keyCode == keyCode

    def hit(self):
        self.hitTime = time()
        if self.firstTime:
            self.timeLeft = 6
        else:
            self.timeLeft = 1
        self.firstTime = False
    
    def isRetired(self):
#        print time() - self.hitTime
        return ((time() - self.hitTime) * 10.0) > self.timeLeft
        
     

class CursesKeyboard(object):
    def __init__(self, keyDownHandler, keyUpHandler):
        self.keyDownHandler = keyDownHandler
        self.keyUpHandler = keyUpHandler

    def setDelay(self, n):
        """
            Set the delay for the next call to stdscr.getch().  This is the time to wait before
            returning with a -1.

            NOTE: This is undocumented behavior...here's the deal.  half-delay mode means getch
            will wait for up to n/10 seconds for a key to be pressed.  If a key was pressed,
            it will return that key code immediately.  If no key is pressed, it will return
            a -1 after n/10 seconds.
        """
        curses.halfdelay(n)

    def read(self):
        curses.wrapper(self.doRead)        
    
    def doRead(self, stdscr):
        curses.raw()
        lastc = None
        delay = 1
        trackedKeys = []
        while 1:

            # Retired key trackers
            retired = [tk for tk in trackedKeys if tk.isRetired() ]
        #    print retired
         #   print trackedKeys
            for r in retired:
                self.keyUpHandler(r.keyCode)

            trackedKeys = [tk for tk in trackedKeys if tk not in retired]
           # print trackedKeys 
            if len(trackedKeys) > 1:     
                delay = max([tk.timeLeft for tk in trackedKeys])
            else:
                delay = 1
            # Set the delay to wait for a key to be pressed.  Do this
            # in all cases, and getch will return -1 when a key has not been pressed
            self.setDelay(delay)

            c = stdscr.getch()

            #if c >0: print c
            found = None
            for tk in trackedKeys:
                if tk.isKey(c):
                    found = tk
                    break
            # getch returned something different
            if not found and c > 0:
                self.keyDownHandler(c)
                tk = TrackedKey(c)
                trackedKeys.append(tk)
                found = tk

            if found:
                found.hit()
            #    print found.isRetired()
            stdscr.refresh()
#            if c != lastc:
 #               # if we were already tracking a key, indicate it is released
  #              # TODO: track multiple keys (with "miss counters" to expire tracks) 
   #             if lastc: 
    #                self.keyUpHandler(lastc)

     #           if c > 0:
      #              self.keyDownHandler(c)
       #             first = True

        #        lastc = c
         #       self.stdscr.refresh();

