#!/usr/bin/env python
# license removed for brevity
import roslib; roslib.load_manifest('simple_face_detection')
import rospy

from std_msgs.msg import String

import cv2
import sys
import logging as log
import datetime as dt

from time import sleep
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError



class detector(object):
    """ Opencv based face detection system. """
    def __init__(self):
      self.pub = rospy.Publisher('chatter', String, queue_size=10)
      self.image_pub = rospy.Publisher("face_detection_image",Image, queue_size=10)
      rospy.init_node('detector', anonymous=True)
      
      self._Cascade_Classifier = "~cascade_classifier"

      # Configure acoustic model
      if rospy.has_param(self._Cascade_Classifier):
	cascPath = rospy.get_param(self._Cascade_Classifier)
      else:
	rospy.logwarn("parameters need to be set to start recognizer.")
	return
      
      self.bridge = CvBridge()
      self.faceCascade = cv2.CascadeClassifier(cascPath)
      
      rospy.loginfo("I see ....")
      self._image_topic = "~image_topic"
      
      
      if rospy.has_param(self._image_topic):
	self.image_sub = rospy.Subscriber(self._image_topic,Image,self.start_application_by_topic)
	rospy.loginfo("I see ....")
      else:
        self.video_capture = cv2.VideoCapture(0)
	#self.start_application()
      
    def start_application_by_topic(self,data): 
      try:
	frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
      except CvBridgeError as e:
	print(e)
      
      gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

      faces = self.faceCascade.detectMultiScale(
	  gray,
	  scaleFactor=1.1,
	  minNeighbors=5,
	  minSize=(30, 30),
	  flags=cv2.cv.CV_HAAR_SCALE_IMAGE
      )

      if len(faces):
	  rospy.loginfo("I see = %d",len(faces))
	  rospy.logdebug("Partial: %s" + str(faces))

      # Draw a rectangle around the faces
      for (x, y, w, h) in faces:
	  cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)

      try:
	self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
      except CvBridgeError as e:
	print(e)
      
      
    def start_application(self):
      rate = rospy.Rate(0.5) # 10hz
      while not rospy.is_shutdown():
	# Capture frame-by-frame
	ret, frame = self.video_capture.read()

	gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

	faces = self.faceCascade.detectMultiScale(
	    gray,
	    scaleFactor=1.1,
	    minNeighbors=5,
	    minSize=(30, 30),
	    flags=cv2.cv.CV_HAAR_SCALE_IMAGE
	)

	if len(faces):
	    rospy.loginfo("I see = %d",len(faces))
	    rospy.logdebug("Partial: %s" + str(faces))

	
	# Draw a rectangle around the faces
	for (x, y, w, h) in faces:
	    cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)

	# Display the resulting frame
	#cv2.imshow('Video', frame)

	#if cv2.waitKey(1) & 0xFF == ord('q'):
	  #break
	
	try:
	  self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
	except CvBridgeError as e:
	  print(e)
      
if __name__ == '__main__':
    try:
	rospy.loginfo("Starting face detector....")
	start = detector()

    except rospy.ROSInterruptException:
        pass
