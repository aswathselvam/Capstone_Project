#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge, CvBridgeError

rospy.init_node('VideoPublisher')

VideoRaw = rospy.Publisher('camera/image_raw', Image, queue_size=10)

cam = cv2.VideoCapture('http://192.168.29.132:8080/video')

while not rospy.is_shutdown():
	meta, frame = cam.read()

    # I want to publish the Canny Edge Image and the original Image
	if meta:
		try:
			VideoRaw.publish(CvBridge().cv2_to_imgmsg(frame))
		except CvBridgeError as e:
			print(e)
	cv2.waitKey(3)

cv2.destroyAllWindows()
