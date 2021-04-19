import numpy as np

p0 = np.array([0.0,0.0,0.0])
p1 = np.array([1.9,0.9,0.0])
p2 = np.array([0.0,0.9,0.0])
p3 = np.array([1.9,0.0,0.2])



a0 = np.concatenate([[1], -2*p0])
a1 = np.concatenate([[1], -2*p1])
a2 = np.concatenate([[1], -2*p2])
a3 = np.concatenate([[1], -2*p3])
A = np.array([a0,a1,a2,a3])
pinvA = np.linalg.pinv(A, rcond=1e-15, hermitian=False)

def trilateration(d0, d1, d2, d3):

    b0 = np.array(d0**2 - p0[0]**2 - p0[1]**2 - p0[2]**2)
    b1 = np.array(d1**2 - p1[0]**2 - p1[1]**2 - p1[2]**2)
    b2 = np.array(d2**2 - p2[0]**2 - p2[1]**2 - p2[2]**2)
    b3 = np.array(d3**2 - p3[0]**2 - p3[1]**2 - p3[2]**2)
    b = np.array([b0,b1,b2,b3])

    y = pinvA.dot(b)

    return y