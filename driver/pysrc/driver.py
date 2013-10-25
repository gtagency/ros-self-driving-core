
import rospy
from core_msgs.msg import *

class TestDriver(object):

    maxSupportedVelocity = 2.2352 # m/s
    maxVelocity = 1 # m/si
    minDuration = 2 #sec
    accel = 50
    timeToTurn = 2 # sec

    def __init__(self):
        rospy.init_node('test_driver')

        # max velocity supported by motor controller/platform
        # velocity is in m/s
        self.maxSupportedVelocity = rospy.get_param("~maxSupportedVelocity", 2.2352) # m/s
        # max velocity the driver is permitted to drive
        # velocity is in m/s
        self.maxVelocity = rospy.get_param("~maxVelocity", 1)
        # acceleration is in % from 0 to 100%
        # for now, assume constant acceleration
        self.accel = rospy.get_param("~accel", 50)
        # min duration is the minimum duration to use
        # in a motor command...this will allow for us to creep/crawl forward if needed
        self.minDuration = rospy.get_param("~minDuration", 2)

        self.timeToTurn = rospy.get_param("~timeToTurn", 2)

        self.planSteps = []

        self.mcPub = rospy.Publish("PhidgetMotor",  MotorCommand)
        self.laPub = rospy.Publish("PhidgetLinear", LinearCommand)

        rospy.Subscribe('plan', Plan, handlePlan)

    def handlePlan(self, data):
        self.planTime  = data.header.stamp
        self.planSteps = data.data
        self.inflatePlan()
        self.runNextStep()

    def inflatePlan(self):
        newSteps = []
        for step in self.planSteps:
            # build a list of sublists, and flatten it later
            if step.angular_displacement != 0:
                newSteps.append(self.splitCurve(step))
            else:
                newSteps.append([step])

        # flatten the list into a plan to execute
        self.planSteps = [step for subSteps in newSteps for step in subSteps]

    def splitCurve(self, step):
        hp = math.hypot(acos(math.pi/2 - step.angular_displacement), step.linear_displacement) 
        arcLen = step.linear_displacement * step.angular_displacement
        step
        secondStep = PlanStep()
        secondStep.

    def runNextStep(self):
        if self.planSteps:
            self.curStep   = self.planSteps.pop(0)

            self.curStep.angle
            # assume max velocity
            speed = maxVelocity
            # compute the duration at this velocity 
            duration = self.curStep.length / maxVelocity 
            # if the duration is too small
            if duration < minDuration:
                # we need to decrease the speed to let us meet the min duration
                duration = minDuration
                ratio = duration/minDuration
                speed *= ratio

            publishMotorCommand(speed, duration) 
        else:
            ros.logwarn("Empty plan, cannot execute step")

    def publishLinearCommand(self, speed, duration):
        msg = LinearCommand()
        msg.speed = speed
        msg.secondsDuration = duration
        msg.acceleration = self.turnAccel
        laPub.publish(msg)

    def publishMotorCommand(self, speed, duration):
        msg = MotorCommand()
        msg.leftSpeed  = speed
        msg.rightSpeed = speed
        msg.secondsDuration = duration
        msg.acceleration = self.accel
        mcPub.publish(msg)

