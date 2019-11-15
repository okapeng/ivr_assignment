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
        print(self.red)
        # self.calculate_transform()

    def callback2(self, pos):
        p = np.reshape(np.array(pos.data), [3, 2])
        self.blue[0] = p[0, 0]
        self.green[0] = p[1, 0]
        self.red[0] = p[2, 0]
        # self.calculate_transform()

    # def calculate_transform(self):
    #     # print self.blue

        

    def transform(self,theta1,theta2,theta3,theta4):
        T10 = self.transform_matrix(np.pi/2, 0, 2, theta1)
        T21 = self.transform_matrix(np.pi/2, 0, 0, theta2)
        T32 = self.transform_matrix(np.pi/2, 3, 0, theta3)
        T43 = self.transform_matrix(0, 2, 0, theta4)
        T = T10.dot(T21).dot(T32).dot(T43)



    def transform_matrix(self, alpha, r, d, theta):
        return np.array([
            [np.cos(theta), -np.sin(theta)*np.cos(alpha), np.sin(theta)*np.sin(alpha), r*np.cos(theta)],
            [np.sin(theta), np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), r*np.sin(theta)],
            [0,np.sin(alpha), np.cos(alpha), d],
            [0,0,0,1]
        ])


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
