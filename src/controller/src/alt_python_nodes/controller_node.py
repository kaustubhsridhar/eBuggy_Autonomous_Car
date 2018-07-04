#!/usr/bin/env python
import roslib
import rospy
from reciever.msg import Cmd
import serial

ser = serial.Serial('/dev/ttyACM0', 9600)

def talker():
  #rate = rospy.Rate(10) # 10hz
  while not rospy.is_shutdown():
    pub = rospy.Publisher('/rec_data', Cmd, queue_size=10)
    data= ser.readline()   # read a '\n' terminated line
    print(data)
    pub.publish(float(data.split(",")[0]), float(data.split(",")[1]), float(data.split(",")[2]))
    #rate.sleep()


if __name__ == '__main__':
  rospy.init_node('talker')
  try:
    talker()
    rospy.spin()
  except rospy.ROSInterruptException:
    pass
