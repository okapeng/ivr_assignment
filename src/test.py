import numpy as np

def transform(theta1,theta2,theta3,theta4):
        # T10 = transform_matrix(0, 0, 0, theta1)
        T21 = transform_matrix(np.pi/2, 0, 2, theta1)
        T32 = transform_matrix(np.pi/2, 0, 0, theta2+np.pi/2)
        T43 = transform_matrix(np.pi/2, 0, 0, theta3+np.pi/2)
        T54 = transform_matrix(np.pi/2, 0, 3, np.pi/2)
        T65 = transform_matrix(-np.pi/2, 0, 2, theta4)
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


X = np.array([0,0,0,1])
# print(T)
T=np.round(transform(0,0, 0, np.pi/4), 3)
print(T)
# print(np.linalg.inv(T))