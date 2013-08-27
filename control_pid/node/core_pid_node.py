import roslib
import rospy
from std_msgs.msg import Float64

# Global publisher for sending angle adjustments
correction_pub = None

# Global error values
old_error = 0
error_integral = 0

# Global gain values, set by service
proportional_gain = 1
integral_gain = 0
derivative_gain = 0


def node():
    correction_pub = rospy.Publisher('control_correction', Float64)
    rospy.Subscriber('control_error', Float64)
    rospy.Service('set_pid_gains', PidGains, set_gains)

    print "PID controller active"
    rospy.spin()


def pid_correction(error_angle):
    global old_error, error_integral, proportional_gain, integral_gain, derivative_gain, correction_pub

    error_differential = error_angle - old_error
    error_integral += error_angle
    old_error = error_angle
    adj_error_angle = proportional_gain * error_angle + derivative_gain * error_differential + integral_gain * error_integral

    correction_pub.publish(adj_error_angle)


def set_gains(req):
    global integral_gain, proportional_gain, derivative_gain
    integral_gain = req.integral_gain
    proportional_gain = req.proportional_gain
    derivative_gain = req.derivative_gain
