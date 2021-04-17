import numpy as np


p1 = np.array([0.0,0.0,0.0])
p2 = np.array([1.9,0.9,0.0])
p3 = np.array([0.0,0.9,0.0])
p4 = np.array([1.9,0.0,0.2])

d1 = 4.6080564040781935
d2 = 6.678773292523172
d3 = 5.194182278874839
d4 = 6.247574883362317


a1 = np.concatenate([[1], -2*p1])
a2 = np.concatenate([[1], -2*p2])
a3 = np.concatenate([[1], -2*p3])
a4 = np.concatenate([[1], -2*p4])
A = np.array([a1,a2,a3,a4])

b1 = np.array(d1**2 - p1[0]**2 - p1[1]**2 - p1[2]**2)
b2 = np.array(d2**2 - p2[0]**2 - p2[1]**2 - p2[2]**2)
b3 = np.array(d3**2 - p3[0]**2 - p3[1]**2 - p3[2]**2)
b4 = np.array(d4**2 - p4[0]**2 - p4[1]**2 - p4[2]**2)
b = np.array([b1,b2,b3,b4])

pinvA = np.linalg.pinv(A, rcond=1e-15, hermitian=False)

y = pinvA.dot(b)

print(y)