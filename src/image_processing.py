#!/usr/bin/env python2.7

import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, Float64
from cv_bridge import CvBridge, CvBridgeError

class image_processer:

  # Defines publisher and subscriber
  def __init__(self):
    # initialize the node named image_processing
    rospy.init_node('image_processing', anonymous=True)
    # initialize a publisher to send images from camera1 to a topic named image_topic1
    self.joint_sub1 = rospy.Subscriber('joint_pos1', Float64MultiArray, self.callback1)
    self.joint_sub2 = rospy.Subscriber('joint_pos2', Float64MultiArray, self.callback2)
    # initialize the bridge between openCV and ROS
    self.bridge = CvBridge()
    self.pos1 = []
    self.pos2 = []

  def callback1(self, pos):
    self.pos1 = np.reshape(pos.data, [4,2])
    self.calculate_transform()

  def callback2(self, pos):
    self.pos2 = np.reshape(pos.data, [4,2])
    self.calculate_transform()

  def calculate_transform(self):
    print(self.pos1)
    print(self.pos2)

# call the class
def main(args):
  ic = image_processer()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)