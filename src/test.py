import numpy as np
import scipy as sci
from scipy.optimize import fsolve, leastsq

def transform(theta1,theta2,theta3,theta4):
        # T10 = transform_matrix(0, 0, 0, theta1)
        T21 = transform_matrix(np.pi/2, 0, 2, theta1)
        T32 = transform_matrix(np.pi/2, 0, 0, theta2+np.pi/2)
        T43 = transform_matrix(np.pi/2, 0, 0, theta3+np.pi/2)
        T54 = transform_matrix(np.pi/2, 0, 3, np.pi/2)
        T65 = transform_matrix(-np.pi/2, 0, 0, theta4)
        T76 = transform_matrix(0, 0, 2, 0)
        T = T21.dot(T32).dot(T43).dot(T54).dot(T65).dot(T76)
        print(np.round(T21.dot(T32).dot(T43).dot(T54),3))
        
        return T

def transform_matrix(alpha, r, d, theta):
    return np.array([
        [np.cos(theta), -np.sin(theta)*np.cos(alpha), np.sin(theta)*np.sin(alpha), r*np.cos(theta)],
        [np.sin(theta), np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), r*np.sin(theta)],
        [0,np.sin(alpha), np.cos(alpha), d],
        [0,0,0,1]
    ])
def F2(x,data):
    return (2*(np.cos(data[2])*np.cos(data[3])*np.cos(x)-np.sin(data[2])*np.sin(x))+data[1]-data[0])

def F1(x, data):
    return ((-3*np.cos(x[0])*np.sin(x[1])*np.cos(x[2])+3*np.sin(x[0])*np.sin(x[2])-data[0]),
            (-3*np.sin(x[0])*np.sin(x[1])*np.cos(x[2])-3*np.cos(x[0])*np.sin(x[2])-data[1]),
            (3*np.cos(x[1])*np.cos(x[2])+2-data[2]))


X = np.array([0,0,0,1])
# print(T)

T=np.round(transform(0,0,np.pi/6,0), 3)
print(T.dot(np.array([0,0,0,1])))
# print(np.linalg.inv(T))
angles = [0,0,0]
r1 = leastsq(F1, angles, args=[-1.5,0, 4.598])
r2 = leastsq(F2, 0, args=[5.598, 4.598, np.pi/6, 0])
print(r2[0][0])
print(np.concatenate([r1[0],r2[0]]))