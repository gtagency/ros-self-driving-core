#!/usr/bin/env python
import roslib
import rospy
from std_msgs.msg import Float64
from control_pid.srv import PidGains

# Global publisher for sending angle adjustments
correction_pub = None

# Global error values
old_error = 0
error_integral = 0

# Global gain values, set by service
proportional_gain = 1
integral_gain = 0
derivative_gain = 0
sampleTime = 1.0
lastTime = 0.0
MaxError = 1.57079633
def node():
    global correction_pub
    rospy.init_node('control_pid')
    correction_pub = rospy.Publisher('control_correction', Float64)
    rospy.Subscriber('control_error', Float64, handle_control_error)
    rospy.Service('set_pid_gains', PidGains, set_gains)
    print "PID controller active"
    rospy.spin()

def handle_control_error(data):
    pid_correction(data.data)

def pid_correction(error_angle):
    global old_error, error_integral, proportional_gain, integral_gain, derivative_gain, correction_pub, sampleTime, lastTime, MaxError
    
    c_time_seconds = rospy.get_time()
    t_diff = c_time_seconds - lastTime
    if(t_diff >= sampleTime):
        error_differential = (error_angle - old_error)/(t_diff)
        error_integral += (error_angle * t_diff)
        if(error_integral > MaxError):
            error_integral = MaxError
        elif(error_integral < -MaxError):
            error_integral = -MaxError
        old_error = error_angle
        lastTime = c_time_seconds
        adj_error_angle = proportional_gain * error_angle + derivative_gain * error_differential + integral_gain * error_integral
        if(adj_error_angle > MaxError):
            adj_error_angle = MaxError
        elif(adj_error_angle < -MaxError):
        print "Recieved error and publishing correction", error_angle,adj_error_angle
        correction_pub.publish(adj_error_angle)
    else:
        print "Too soon"


def set_gains(req):
    global integral_gain, proportional_gain, derivative_gain
    print "Setting gains", req.proportional_gain, req.integral_gain, req.derivative_gain
    integral_gain = req.integral_gain
    proportional_gain = req.proportional_gain
    derivative_gain = req.derivative_gain

if __name__ == "__main__":
    node()
