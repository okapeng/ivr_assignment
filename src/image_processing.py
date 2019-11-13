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
        self.yellow = np.ones(4)
        self.bridge = CvBridge()
        self.blue = np.ones(4)
        self.red = np.ones(4)
        self.green = np.ones(4)

    def callback1(self, pos):
        p = np.reshape(np.array(pos.data), [3, 2])
        self.blue[2] = p[0, 1]
        self.green[2] = p[1, 1]
        self.red[2] = p[2, 1]
        self.blue[1] = p[0, 0]
        self.green[1] = p[1, 0]
        self.red[1] = p[2, 0]
        self.calculate_transform()

    def callback2(self, pos):
        p = np.reshape(np.array(pos.data), [3, 2])
        self.blue[0] = p[0, 0]
        self.green[0] = p[1, 0]
        self.red[0] = p[2, 0]
        self.calculate_transform()

    def calculate_transform(self):
        # print self.blue
        self.transform_matrix()


    def transform_matrix(self):
        def vector_product(v1,v2):
          return np.array([v1[0]*v2.T, v1[1]*v2.T,v1[2]*v2.T,v1[3]*v2.T])
        # print M
        # inv =  np.linalg.inv(vector_product(self.blue,self.blue))
        print vector_product(self.blue,self.blue)
        # T31 = np.dot(vector_product(self.green,self.blue) , inv)
        # print T31
        # print inv


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
