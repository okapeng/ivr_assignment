#!/usr/bin/env python2.7

import roslib
import sys
import rospy
import cv2
import numpy as np
from scipy.optimize import leastsq, least_squares, fsolve
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
        # initialize a publisher to send robot end-effector position
        self.end_effector_actual_pub = rospy.Publisher("end_effector_position",Float64MultiArray, queue_size=10)
        # initialize a publisher to send desired trajectory
        self.target_trajectory_pub = rospy.Publisher("target_trajectory",Float64MultiArray, queue_size=10)
        # initialize a publisher to send joints' angular position to the robot
        # self.target_position_pub = rospy.Publisher("/target/target_detected",Float64MultiArray, queue_size=10)
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
        self.error = np.array([0.0,0.0,0.0], dtype='float64')
        self.error_d = np.array([0.0,0.0,0.0], dtype='float64')
        self.joint1_d = Float64()
        self.joint2_d = Float64()
        self.joint3_d = Float64()
        self.joint4_d = Float64()

    def control_closed(self):
        # P gain
        K_p = np.array([[10,0,0], [0,10,0], [0,0,10]])
        # D gain
        K_d = np.array([[0.1,0,0],[0,0.1,0],[0,0,0.1]])
        # estimate time step
        cur_time = np.array([rospy.get_time()])
        dt = cur_time - self.time_previous_step
        self.time_previous_step = cur_time
        # robot end-effector position
        pos = self.red[:3]
        # desired trajectory
        pos_d= self.target[:3]
        # estimate derivative of error
        self.error_d = ((pos_d - pos) - self.error)/dt
        # estimate error
        self.error = pos_d-pos
        q1 = self.estimate_joint_angles()
        q = [self.joint1, self.joint2, self.joint3, self.joint4] # estimate initial value of joints'
        print("Joint angles: ", q)
        J_inv = np.linalg.pinv(self.calculate_jacobian(self.joint1,self.joint2,self.joint3,self.joint4))  # calculating the psudeo inverse of Jacobian
        dq_d =np.dot(J_inv, ( np.dot(K_d,self.error_d.transpose()) + np.dot(K_p,self.error.transpose()) ) )  # control input (angular velocity of joints)
        q_d = q + (dt * dq_d)  # control input (angular position of joints)
        return q_d



    def callback1(self, pos):
        self.p1 = np.reshape(np.array(pos.data), [4, 2])
        self.calculate_transform()

    def callback2(self, pos):
        self.p2 = np.reshape(np.array(pos.data), [4, 2])
        self.calculate_transform()



    def calculate_transform(self):
        if self.p1 is not None and self.p2 is not None:
            self.blue[1] = -self.p1[0, 0]
            self.green[1] = -self.p1[1, 0]
            self.red[1] = -self.p1[2, 0]
            self.target[1] = -self.p1[3, 0]

            self.blue[0] = -self.p2[0, 0]
            self.green[0] = -self.p2[1, 0]
            self.red[0] = -self.p2[2, 0]
            self.target[0] = -self.p2[3, 0]

            self.blue[2] = self.p2[0, 1] if self.p2[0, 1] == 0 else self.p1[0, 1]
            self.green[2] = self.p2[1, 1] if self.p2[1, 1] == 0 else self.p1[1, 1]
            self.red[2] = self.p2[2, 1] if self.p2[2, 1] == 0 else self.p1[2, 1]
            self.target[2] = self.p2[3, 1] if self.p2[3, 1] == 0 else self.p1[3, 1]

            self.check_position(self.red, self.blue)
            self.check_position(self.green, self.blue)
            self.check_position(self.red, self.green)
            self.target_detected = Float64MultiArray()
            self.target_detected.data = [self.target[0],self.target[1],self.target[2]]
            self.target_trajectory_pub.publish(self.target_detected)

            # print("green: ", self.green)
            # print("red  : ", self.red)
            # print("target  : ", self.target)
            self.p1 = None
            self.p2 = None
            T = self.forward_kinematic()
            print("end_effector_pos ", np.round(T.dot(np.array([0,0,0,1])),3))
            q_d = self.control_closed()
            
            self.joint1_d.data,self.joint2_d.data,self.joint3_d.data,self.joint4_d.data = q_d[0],q_d[1],q_d[2],q_d[3]
            self.robot_joint1_pub.publish(self.joint1_d)
            self.robot_joint2_pub.publish(self.joint2_d)
            self.robot_joint3_pub.publish(self.joint3_d)
            self.robot_joint4_pub.publish(self.joint4_d)
            end_effector_actual = Float64MultiArray()
            end_effector_actual.data = self.red
            self.end_effector_actual_pub.publish(end_effector_actual)

    def check_position(self, j1, j2):
        if (np.absolute(j1[2] - j2[2]) < 0.1):
            j1[0] = j1[0] if j1[0] != 0 else j2[0]
            j2[0] = j2[0] if j2[0] != 0 else j1[0]
            j1[1] = j1[1] if j1[1] != 0 else j2[1]
            j2[1] = j2[1] if j2[1] != 0 else j1[1]

    def transform_matrix(self, alpha, r, d, theta):
        return np.array([
            [np.cos(theta), -np.sin(theta)*np.cos(alpha), np.sin(theta)*np.sin(alpha), r*np.cos(theta)],
            [np.sin(theta), np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), r*np.sin(theta)],
            [0,np.sin(alpha), np.cos(alpha), d],
            [0,0,0,1]
        ])

    def forward_kinematic(self):
        self.estimate_joint_angles()
        T21 = self.transform_matrix(np.pi/2, 0, 2, self.joint1+np.pi/2)
        T32 = self.transform_matrix(np.pi/2, 0, 0, self.joint2+np.pi/2)
        T43 = self.transform_matrix(-np.pi/2, 3, 0, self.joint3)
        T54 = self.transform_matrix(0, 2, 0, self.joint4)
        return T21.dot(T32).dot(T43).dot(T54)



    def estimate_joint_angles(self):
        def F1(x, data):
            return ((3*np.sin(x[0])*np.sin(x[1])*np.cos(x[2])+3*np.cos(x[0])*np.sin(x[2])-data[0]),
                    (-3*np.cos(x[0])*np.sin(x[1])*np.cos(x[2])-3*np.sin(x[0])*np.sin(x[2])-data[1]),
                    (3*np.cos(x[1])*np.cos(x[2])+2-data[2]),
                    (2*(np.cos(x[1])*np.cos(x[2])*np.cos(x[3])-np.sin(x[1])*np.sin(x[3]))+data[2]-data[3]))
        # def F1(x, gx,gy,gz,rz):
        #     return np.array([(3*np.sin(x[0])*np.sin(x[1])*np.cos(x[2])+3*np.cos(x[0])*np.sin(x[2])-gx),
        #             (-3*np.cos(x[0])*np.sin(x[1])*np.cos(x[2])-3*np.sin(x[0])*np.sin(x[2])-gy),
        #             (3*np.cos(x[1])*np.cos(x[2])+2-gz),
        #             (2*(np.cos(x[1])*np.cos(x[2])*np.cos(x[3])-np.sin(x[1])*np.sin(x[3]))+gz-rz)])
        # (np.square(x[0])+np.square(x[1])+np.square(x[2])+np.square(x[3]))
        solution = leastsq(F1, [0,0,0,0], args=[self.green[0],self.green[1], self.green[2], self.red[2]])
        # solution = least_squares(F1, [0, 0, 0, 0], bounds=([-np.pi / 2, np.pi / 2]),max_nfev=300,
        #                          args=(self.green[0], self.green[1], self.green[2], self.red[2]))
        if(reduce(lambda x,y: x or y>np.pi or y<-np.pi, solution[0], False)):
            return
        self.joint1, self.joint2, self.joint3, self.joint4 = np.round(solution[0] , 2)

        # if(self.joint4 > np.pi/2 or self.joint4 < -np.pi/2):
        #     solution = leastsq(F2, [0,0,0], args=[self.green[0],self.green[1], self.green[2]])
        #     self.joint1, self.joint2, self.joint3 = np.round(solution[0] , 2)
        #     self.joint4 = 0
        # self.joint1, self.joint2, self.joint3, self.joint4 = np.round(solution.x,2)


    def calculate_jacobian(self,t1,t2,t3,t4):
        # calculate jacobian matrix
        def cos(x):
            return np.cos(x)

        def sin(x):
            return np.sin(x)

        j21 = - 3 * (cos(t3) * sin(t1) * sin(t2) + cos(t1) * sin(t3)) - 2 * (
                cos(t3) * cos(t4) * sin(t1) * sin(t2) + cos(t1) * cos(t4) * sin(t3) + cos(t2) * sin(t1) * sin(t4))
        j22 = 3 * cos(t1) * cos(t2) * cos(t3) - 2 * (
                    -cos(t1) * cos(t2) * cos(t3) * cos(t4) + cos(t1) * sin(t2) * sin(t4))
        j23 = -3 * (cos(t3) * sin(t1) + cos(t1) * sin(t2) * sin(t3)) - 2 * (
                cos(t3) * cos(t4) * sin(t1) + cos(t1) * cos(t4) * sin(t2) * sin(t3))
        j24 = -2 * (-cos(t1) * cos(t2) * cos(t4) + cos(t1) * cos(t3) * sin(t2) * sin(t4) - sin(t1) * sin(t3) * sin(t4))

        j11 = 3 * (-cos(t1) * cos(t3) * sin(t2) + sin(t1) * sin(t3)) + 2 * (
                -cos(t1) * cos(t3) * cos(t4) * sin(t2) + cos(t4) * sin(t1) * sin(t3) - cos(t1) * cos(t2) * sin(t4))
        j12 = -3 * cos(t2) * cos(t3) * sin(t1) + 2 * (
                    -cos(t2) * cos(t3) * cos(t4) * sin(t1) + sin(t1) * sin(t2) * sin(t4))
        j13 = 3 * (-cos(t1) * cos(t3) + sin(t1) * sin(t2) * sin(t3)) + 2 * (
                -cos(t1) * cos(t3) * cos(t4) + cos(t4) * sin(t1) * sin(t2) * sin(t3))
        j14 = 2 * (-cos(t2) * cos(t4) * sin(t1) + cos(t3) * sin(t1) * sin(t2) * sin(t4) + cos(t1) * sin(t3) * sin(t4))

        j31 = 0
        j32 = -3 * cos(t3) * sin(t2) + 2 * (-cos(t3) * cos(t4) * sin(t2) - cos(t2) * sin(t4))
        j33 = -3 * cos(t2) * sin(t3) - 2 * cos(t2) * cos(t4) * sin(t3)
        j34 = 2 * (-cos(t4) * sin(t2) - cos(t2) * cos(t3) * sin(t4))
        return np.array([
            [-j11, -j12, -j13, -j14],
            [-j21, -j22, -j23, -j24],
            [j31, j32, j33, j34]
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
