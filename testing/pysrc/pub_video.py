#!/usr/bin/env python
import rospy
import cv
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from testing.srv import PublishImage

image_pub = None

bridge = CvBridge()
cam = None
frameCount = 0
def nextFrame():
    global cam
    if cam and cam.isOpened():
        global frameCount
        cv2.namedWindow("Test",0)

        success, M = cam.read();
        frameCount += 1

        cv_image = cv.fromarray(M)
        image_pub.publish(bridge.cv_to_imgmsg(cv_image, "bgr8"))
 
def publish_from_file(filename):
    global bridge,image_pub,cam
    cam = cv2.VideoCapture()
    cam.open(filename)
    print filename, "publishing."
    
def handle_publish_image(req):
    publish_from_file(req.filename)

def node():
    global image_pub
    rospy.init_node('pub_video')
    filename = rospy.get_param("~video_file", None)
#    s = rospy.Service('publish_image', PublishImage, handle_publish_image)
    image_pub = rospy.Publisher('image_raw', Image, latch=True)
    print "Ready to publish videos."
    if filename:
        publish_from_file(filename)
    r = rospy.Rate(30) #30fps
    while not rospy.is_shutdown():
        r.sleep()
        nextFrame()
    rospy.spin()

if __name__ == '__main__':
    node()
