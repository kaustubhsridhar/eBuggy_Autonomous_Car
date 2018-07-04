#!/usr/bin/env python

import roslib
import sys
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from cam_data.msg import Yaw

#this node is used only to see debugging images at one of the tree topics using one of the three subs below

class image_converter:

  def __init__(self):
    self.bridge = CvBridge()
    #self.image_sub = rospy.Subscriber("/lane_model/lane_markings",Image,self.callback)
    #self.image_sub = rospy.Subscriber("/lane_model/lane_model_image",Image,self.callback)
    self.image_sub = rospy.Subscriber("/lane_model/ransac",Image,self.callback)
    #self.image_sub = rospy.Subscriber("/csi_cam/image_raw",Image,self.callback)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    height, width, channels = cv_image.shape
    crop_img = cv_image[1:height][1:width] # crop image


    cv2.imshow("Image window", np.hstack([crop_img])) 	
    cv2.waitKey(1)				

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

