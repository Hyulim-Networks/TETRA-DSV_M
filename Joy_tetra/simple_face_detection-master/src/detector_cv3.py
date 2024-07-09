#!/usr/bin/env python
import roslib
roslib.load_manifest('simple_face_detection')
import rospy
import cv2
import sys

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


import logging as log
import datetime as dt

from time import sleep

pubX = rospy.Publisher('CV3_Result_X', String, queue_size=10)
pubY = rospy.Publisher('CV3_Result_Y', String, queue_size=10)


class image_converter:

  def __init__(self):
    rospy.init_node('image_converter', anonymous=True)
    
    rospy.loginfo("recognizer started")

    #Configure acoustic model
    self._Cascade_Classifier = "~cascade_classifier"
    if rospy.has_param(self._Cascade_Classifier):
      cascPath = rospy.get_param(self._Cascade_Classifier)
    else:
      rospy.logwarn("parameters need to be set to start recognizer.")
      return
    
    
    self.bridge = CvBridge()
    self.faceCascade = cv2.CascadeClassifier(cascPath)
    
    #Where to publish
    self._output_image_topic = "~image_topic_output"
    print rospy.has_param(self._output_image_topic)
    if rospy.has_param(self._output_image_topic):
      output_image_topic = rospy.get_param(self._output_image_topic)
      self.image_pub = rospy.Publisher(output_image_topic,Image, queue_size=10)
  
      
    #Where to subscribe
    self._input_image_topic = "~image_topic_input"
    print rospy.has_param(self._input_image_topic)
    if rospy.has_param(self._input_image_topic):
      input_image_topic = rospy.get_param(self._input_image_topic)
      self.image_sub = rospy.Subscriber(input_image_topic, Image, self.callback)

    

  def callback(self,data):
    try:
      frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
      flipHorizontal = cv2.flip(frame, 1) # Image Mirror add
    except CvBridgeError as e:
      print(e)

    #(rows,cols,channels) = cv_image.shape
    #if cols > 60 and rows > 60 :
      #cv2.circle(cv_image, (50,50), 10, 255)
    gray = cv2.cvtColor(flipHorizontal, cv2.COLOR_BGR2GRAY)

    faces = self.faceCascade.detectMultiScale(
        gray,
        scaleFactor=1.3,
	minNeighbors=5,
        minSize=(3, 3)
    )

#-------------------------------------------------
    #if (len(faces) == 1): rospy.loginfo("Result:%s", str(faces))
  

    # Draw a rectangle around the faces
    for (x, y, w, h) in faces:
	    cv2.rectangle(flipHorizontal, (x, y), (x+w, y+h), (0, 255, 0), 2)
	    cv2.circle(flipHorizontal, (x+(w/2), y+(h/2)), 5, 255,-1)
      

    #cv2.imshow("Image window", flipHorizontal)
    cv2.waitKey(3)

    if (len(faces) == 1): 
      rospy.loginfo("%d,%d", x, y)
      Result_str1 = "%d" % (x)
      Result_str2 = "%d" % (y)
      pubX.publish(Result_str1)
      pubY.publish(Result_str2)


    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(flipHorizontal, "bgr8"))

    except CvBridgeError as e:
      print(e)

#def main(args):
  #rospy.loginfo("recognizer started")
  #print "................................................"
  #ic = image_converter()
  #rospy.init_node('image_converter', anonymous=True)
  #try:
    #rospy.spin()
  #except KeyboardInterrupt:
    #print("Shutting down")
  #cv2.destroyAllWindows()

#if __name__ == '__main__':
    #main(sys.argv)
    
if __name__ == '__main__':
  rospy.loginfo("simple_face_detection ...........")
  print "................................................"
  ic = image_converter()
  
  
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()
 
