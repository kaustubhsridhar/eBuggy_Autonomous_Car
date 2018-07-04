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

    cv2.circle(extraction,(320, 240), 2,(0,0,255),3) #marking center of 640x480 image as red dot (BGR)

    # for obtaining lines in the red-line extracted frames of video
    gray = cv2.cvtColor(extraction,cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray,50,150,apertureSize = 3)
    sum_left_d = 0; sum_right_d = 0; n_left = 0; n_right = 0
    try:
      lines = cv2.HoughLines(edges,1,np.pi/180,40)[0]		# tune fourth argument to houghlines to change what length of line is seen
      for rho,theta in lines:
        a = np.cos(theta)
        b = np.sin(theta)
        x0 = a*rho
        y0 = b*rho
        x1 = int(x0 + 1000*(-b))
        y1 = int(y0 + 1000*(a))
        x2 = int(x0 - 1000*(-b))
        y2 = int(y0 - 1000*(a))
        cv2.line(extraction,(x1,y1),(x2,y2),(255,0,0),2) #BGR (255,0,0) = Blue
        Dy = y2-y1
        Dx = x2-x1
        dist = abs(Dy*(320) - Dx*(240) - x1*y2 + x2*y1)/pow( Dy**2 + Dy**2,0.5) #distance from center of image which is (320,240)
        print(dist, x1, y1, x2, y2)

        if ((x1+x2)/2>320):
          sum_right_d = sum_right_d + dist
          n_right = n_right+1
        if ((x1+x2)/2<320):
          sum_left_d = sum_left_d + dist
          n_left = n_left+1
    except Exception as e:
      print 'There is no line to be detected!'
    if (n_left != 0 and n_right != 0):
      print("avg left dist:  ", sum_left_d/n_left, "    avg right dist:  ", sum_right_d/n_right)
    

    m = cv2.moments(mask, False)
    try:
      x, y = m['m10']/m['m00'], m['m01']/m['m00']
    except ZeroDivisionError:
      x, y = height/2, width/2
    cv2.circle(extraction,(int(x), int(y)), 2,(0,255,0),3) #marking centroid on image in Green (BGR)
    
    # comment this if you dont want to see window during execution
    cv2.imshow("Image window", np.hstack([crop_img,extraction])) 		#  ORIGINAL
    cv2.waitKey(1)  # delay for one millisecond					#  original

    # cv2.namedWindow("output", cv2.WINDOW_NORMAL)        # Create window with freedom of dimensions
    # im = np.hstack([crop_img,extraction])               # Read image
    # imS = cv2.resize(im, (640, 480))                    # Resize image
    # cv2.imshow("output", imS)                           # Show image
    # cv2.waitKey(0.1)                                    # Display the image for 1

    
    yaw = 1600 + (x - width/2) * 1.5

    # print "center=" + str(width/2) + "point=" + str(x) + "yaw=" +  str(yaw)

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

