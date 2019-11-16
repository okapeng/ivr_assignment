#!/usr/bin/env python2.7

import roslib
import sys
import rospy
import cv2
import numpy as np
from scipy.optimize import leastsq
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
        # initialize a publisher to send robot end-effector position
        self.end_effector_pub = rospy.Publisher("end_effector_prediction",Float64MultiArray, queue_size=10)
        # initialize a publisher to send desired trajectory
        self.target_trajectory_pub = rospy.Publisher("target_trajectory",Float64MultiArray, queue_size=10)
        # initialize a publisher to send joints' angular position to the robot
        self.robot_joint1_pub = rospy.Publisher("/robot/joint1_position_controller/command", Float64, queue_size=10)
        self.robot_joint2_pub = rospy.Publisher("/robot/joint2_position_controller/command", Float64, queue_size=10)
        self.robot_joint3_pub = rospy.Publisher("/robot/joint3_position_controller/command", Float64, queue_size=10)
        self.robot_joint4_pub = rospy.Publisher("/robot/joint4_position_controller/command", Float64, queue_size=10)
        # initialize the bridge between openCV and ROS
        self.bridge = CvBridge()
        self.yellow = np.zeros(4)
        self.blue = np.ones(4)
        self.red = np.ones(4)
        self.green = np.ones(4)
        self.target = np.ones(4)
        self.p1 = None
        self.p2 = None
        # record the begining time
        self.time_trajectory = rospy.get_time()
        # initialize errors
        self.time_previous_step = np.array([rospy.get_time()], dtype='float64')     
        # initialize error and derivative of error for trajectory tracking  
        self.error = np.array([0.0,0.0], dtype='float64')  
        self.error_d = np.array([0.0,0.0], dtype='float64') 

    def control_closed(self,image):
        # P gain
        K_p = np.array([[10,0,0], [0,10,0], [0,0,10]])
        # D gain
        K_d = np.array([[0.1,0,0],[0,0.1,0],[0,0,0.1]])
        # estimate time step
        cur_time = np.array([rospy.get_time()])
        dt = cur_time - self.time_previous_step
        self.time_previous_step = cur_time
        # robot end-effector position
        pos = self.red
        # desired trajectory
        pos_d= self.target 
        # estimate derivative of error
        self.error_d = ((pos_d - pos) - self.error)/dt
        # estimate error
        self.error = pos_d-pos
        q = self.estimate_joint_angles() # estimate initial value of joints'
        J_inv = np.linalg.pinv(self.calculate_jacobian(image))  # calculating the psudeo inverse of Jacobian
        dq_d =np.dot(J_inv, ( np.dot(K_d,self.error_d.transpose()) + np.dot(K_p,self.error.transpose()) ) )  # control input (angular velocity of joints)
        q_d = q + (dt * dq_d)  # control input (angular position of joints)
        return q_d


    def calculate_jacobian(self):
        # Todo
        return

    def estimate_joint_angles(self):
        # def F2(x,data):
        #     return (2*(np.cos(data[2])*np.cos(data[3])*np.cos(x)-np.sin(data[2])*np.sin(x))+data[1]-data[0])
        def F2(x,data):
            return (2*(np.cos(data[2])*np.cos(data[3])*np.cos(x)-np.sin(data[2])*np.sin(x))+data[1]-data[0])
        # def F1(x, data):
        #     return ((-3*np.sin(x[0])*np.sin(x[1])*np.cos(x[2])-3*np.cos(x[0])*np.sin(x[2])-data[0]),
        #             (-3*np.cos(x[0])*np.sin(x[1])*np.cos(x[2])+3*np.sin(x[0])*np.sin(x[2])-data[1]),
        #             (3*np.cos(x[1])*np.cos(x[2])+2-data[2]))
        # def F1(x, data):
        #     return ((-3*np.sin(x[0])*np.sin(x[1])*np.cos(x[2])-3*np.cos(x[0])*np.sin(x[2])-data[0]),
        #             (-3*np.cos(x[0])*np.sin(x[1])*np.cos(x[2])+3*np.sin(x[0])*np.sin(x[2])-data[1]),
        #             (3*np.cos(x[1])*np.cos(x[2])+2-data[2]))
        def F1(x, data):
            return ((-3*np.sin(x[0])*np.sin(x[1])*np.cos(x[2])-3*np.cos(x[0])*np.sin(x[2])-data[0]),
                    (-3*np.cos(x[0])*np.sin(x[1])*np.cos(x[2])+3*np.sin(x[0])*np.sin(x[2])-data[1]),
                    (3*np.cos(x[1])*np.cos(x[2])+2-data[2]),
                    (2*(np.cos(x[1])*np.cos(x[2])*np.cos(x[3])-np.sin(x[1])*np.sin(x[3]))+data[2]-data[3]))
        solution = leastsq(F1, [0,0,0,0], args=[self.green[0],self.green[1], self.green[2], self.red[2]])
        self.joint1, self.joint2, self.joint3, self.joint4 = list(np.round(solution[0],1))
        # solution = leastsq(F1, [0,0,0], args=[self.green[0],self.green[1], self.green[2]])
        # self.joint1, self.joint2, self.joint3 = list(np.round(solution[0],1))
        # solution = leastsq(F2, 0, args=[self.red[2], self.green[2], self.joint2, self.joint3])
        # self.joint4 = np.round(solution[0][0],1)
        print(self.joint1, self.joint2, self.joint3, self.joint4)

    def callback1(self, pos):
        self.p1 = pos.data
        self.calculate_transform()

    def callback2(self, pos):
        self.p2 = pos.data
        # self.calculate_transform()


    def calculate_transform(self):
        if self.p1 is not None and self.p2 is not None:
            p = np.reshape(np.array(self.p1), [4, 2])
            self.blue[1] = -p[0, 0]
            self.green[1] = -p[1, 0]
            self.red[1] = -p[2, 0]
            self.target[1] = -p[3, 0]

            p = np.reshape(np.array(self.p2), [4, 2])
            self.blue[2] = p[0, 1]
            self.green[2] = p[1, 1]
            self.red[2] = p[2, 1]
            self.target[2] = p[3, 1]
            self.blue[0] = -p[0, 0]
            self.green[0] = -p[1, 0]
            self.red[0] = -p[2, 0]
            self.target[0] = -p[3, 0]

            # print("green: ", self.green)
            # print("red  : ", self.red)
            print("target  : ", self.target)
            self.p1 = None
            self.p2 = None
            # self.estimate_joint_angles()

  

    def transform(theta1,theta2,theta3,theta4):
        # T10 = transform_matrix(0, 0, 0, theta1)
        T21 = transform_matrix(np.pi/2, 0, 2, theta1)
        T32 = transform_matrix(np.pi/2, 0, 0, theta2+np.pi/2)
        T43 = transform_matrix(np.pi/2, 0, 0, theta3+np.pi/2)
        T54 = transform_matrix(np.pi/2, 0, 3, np.pi/2)
        T65 = transform_matrix(-np.pi/2, 0, 0, theta4)
        T76 = transform_matrix(0, 0, 2, 0)
        T = T21.dot(T32).dot(T43).dot(T54).dot(T65).dot(T76)
        
        return T



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
