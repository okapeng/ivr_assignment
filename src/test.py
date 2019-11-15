import numpy as np
import math

def transform(theta1,theta2,theta3,theta4):
        # T10 = transform_matrix(0, 0, 0, theta1)
        T21 = transform_matrix(np.pi/2, 0, 2, theta1)
        T32 = transform_matrix(np.pi/2, 0, 0, theta2+np.pi/2)
        T43 = transform_matrix(np.pi/2, 0, 0, theta3+np.pi/2)
        T54 = transform_matrix(np.pi/2, 0, 3, np.pi/2)
        T65 = transform_matrix(-np.pi/2, 0, 0, theta4)
        T76 = transform_matrix(0, 0, 2, 0)
        T = T21.dot(T32).dot(T43).dot(T54).dot(T65).dot(T76)
        print(np.round(T21.dot(T32).dot(T43), 3))
        print(np.round(T54,3))
        # print(math)
        # print(T32)
        # print(T21.dot(T32))
        # T10 = transform_matrix(np.pi/2, 0, 2, theta1+np.pi/2)
        # T21 = transform_matrix(np.pi/2, 0, 0, theta2+np.pi/2)
        # T32 = transform_matrix(np.pi/2, 0, 0, theta3)
        # T43 = transform_matrix(0, 2, 0, -theta4)
        # T = T10.dot(T21).dot(T32).dot(T43)
        return T

def transform_matrix(alpha, r, d, theta):
    return np.array([
        [math.cos(theta), -math.sin(theta)*math.cos(alpha), math.sin(theta)*math.sin(alpha), r*math.cos(theta)],
        [math.sin(theta), math.cos(theta)*math.cos(alpha), -math.cos(theta)*math.sin(alpha), r*math.sin(theta)],
        [0,math.sin(alpha), math.cos(alpha), d],
        [0,0,0,1]
    ])


X = np.array([0,0,0,1])
# print(T)
T=np.round(transform(1.5, 0, 0, 0), 3)
print(T)
# print(np.linalg.inv(T))