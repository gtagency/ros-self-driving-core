#!/usr/bin/env python
import roslib; roslib.load_manifest('corobot_pid')
import rospy
import cv
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from corobot_pid.srv import PublishImage

image_pub = None

bridge = CvBridge()

def handle_publish_image(req):
    global bridge,image_pub
    cv_image = cv.fromarray(cv2.imread(req.filename))
    image_pub.publish(bridge.cv_to_imgmsg(cv_image, "bgr8"))
    print req.filename, "published."

def node():
    global image_pub
    rospy.init_node('pub_image')
    s = rospy.Service('publish_image', PublishImage, handle_publish_image)
    image_pub = rospy.Publisher('image_raw', Image)
    print "Ready to publish images."
    rospy.spin()

if __name__ == '__main__':
    node()
