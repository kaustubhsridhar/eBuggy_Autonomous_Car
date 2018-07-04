#!/usr/bin/env python

import roslib
import sys
import rospy
import cv2
import numpy as np
#import mavros
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image
#from mavros.msg import OverrideRCIn
from cam_data.msg import Yaw

class image_converter:

  def __init__(self):

    self.bridge = CvBridge()
    #self.pub = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=10)
    self.pub = rospy.Publisher('/cam_yaw', Yaw, queue_size=10)
    #self.image_sub = rospy.Subscriber("/rover/front/image_front_raw",Image,self.callback)
    self.image_sub = rospy.Subscriber("/csi_cam/image_raw",Image,self.callback)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    height, width, channels = cv_image.shape
    #crop_img = cv_image[200:(height)/2+150][1:width]
    crop_img = cv_image[1:height][1:width] # crop image

    hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV) # Convert BGR to HSV

    lower = np.array([160, 100, 100], dtype = "uint8") # define range of red color in HSV
    upper = np.array([179, 255, 255], dtype = "uint8")

    mask = cv2.inRange(hsv, lower, upper) # Threshold the HSV image to get only red colors
    
    # mask = cv2.erode(mask, None, iterations=2) # a series of dilations and erosions to remove any small blobs left in the mask
    # mask = cv2.dilate(mask, None, iterations=2) 

    extraction = cv2.bitwise_and(crop_img, crop_img, mask = mask)
    m = cv2.moments(mask, False)
    try:
      x, y = m['m10']/m['m00'], m['m01']/m['m00']
    except ZeroDivisionError:
      x, y = height/2, width/2
    cv2.circle(extraction,(int(x), int(y)), 2,(0,255,0),3)

    cv2.imshow("Image window", np.hstack([crop_img,extraction]))
    cv2.waitKey(1)

    yaw = 1600 + (x - width/2) * 1.5
    print "center=" + str(width/2) + "point=" + str(x) + "yaw=" +  str(yaw)
    #throttle = 1900

    if (yaw > 1900):
      yaw = 1900
    elif (yaw < 1300):
      yaw = 1300


    #msg = OverrideRCIn()

    #msg.channels[0] = yaw
    #msg.channels[1] = 0
    #msg.channels[2] = throttle
    #msg.channels[3] = 0
    #msg.channels[4] = 0
    #msg.channels[5] = 0
    #msg.channels[6] = 0
    #msg.channels[7] = 0
    
    self.pub.publish(yaw)

def main(args):
  rospy.init_node('erle_rover_followline', anonymous=True)
  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

