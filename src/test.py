import numpy as np

def transform(theta1,theta2,theta3,theta4):
        T10 = transform_matrix(np.pi/2, 0, 2, theta1+np.pi/2)
        T21 = transform_matrix(np.pi/2, 0, 0, theta2+np.pi/2)
        T32 = transform_matrix(np.pi/2, 3, 0, theta3)
        T43 = transform_matrix(0, 2, 0, -theta4)
        T = T10.dot(T21).dot(T32).dot(T43)
        return T

def transform_matrix(alpha, r, d, theta):
    return np.array([
        [np.cos(theta), -np.sin(theta)*np.cos(alpha), np.sin(theta)*np.sin(alpha), r*np.cos(theta)],
        [np.sin(theta), np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), r*np.sin(theta)],
        [0,np.sin(alpha), np.cos(alpha), d],
        [0,0,0,1]
    ])

def transform_matrix1(alpha, r, d, theta):
    return np.array([

    ])

X = np.array([0,0,0,1])
# print(T)
print(transform(0,0,1,1))