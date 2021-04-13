#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge, CvBridgeError

rospy.init_node('VideoPublisher')

VideoRaw = rospy.Publisher('camera/image_raw', Image, queue_size=10)
src_motog5splus='http://192.168.29.132:8080/video'
src_iphone="http://192.168.29.202:4747/mjpegfeed"
src_s8="http://192.168.29.52:8080/video"
cam = cv2.VideoCapture(src_motog5splus)

while not rospy.is_shutdown():
	meta, frame = cam.read()

    # I want to publish the Canny Edge Image and the original Image
	if meta:
		try:
			#frame=cv2.resize(frame,(720,480))
			VideoRaw.publish(CvBridge().cv2_to_imgmsg(frame,"bgr8"))
		except CvBridgeError as e:
			print(e)
	cv2.waitKey(3)

cv2.destroyAllWindows()
