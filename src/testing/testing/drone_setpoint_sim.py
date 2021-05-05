import functions
import numpy as np
import matplotlib.pyplot as plt 


dt = 0.1

A = np.matrix([[1, 0],[0, 1]])
B = np.matrix([[dt, 0],[0, dt]])
Q = np.matrix([[1, 0],[0, 1]])
R = np.matrix([[1, 0],[0, 1]])
K = functions.DLQR_optimizer(A, B, Q, R)

kp = 0.5
ki = 0.0001
kd = 0.0001
e_old = np.array([0,0])
int_e = np.array([0,0])
u_max = float("inf")
u_min = - float("inf")
int_max = float("inf")

t_sim = 60
time = np.linspace(0, t_sim, t_sim/0.1, endpoint=True)
xk = np.matrix("50; 20")

EX = []
EY = []
UX = []
UY = []

for t in time:

    uk = K*xk
    # uk, int_e, e_dot, e_old = functions.PID(kp, ki, kd, xk, e_old, int_e, u_max, u_min, int_max)
    
    xk = A*xk + B*uk

    EX.append(xk[0,0])
    EY.append(xk[1,0])
    UX.append(uk[0,0])
    UY.append(uk[1,0])
    
    

plt.plot(time, EX, label="EX error, meters")
plt.plot(time, EY, label="EY error, meters")
plt.plot(time, UX, label="X velocity, meters")
plt.plot(time, UY, label="Y velocity, meters")


plt.legend(loc='upper right')
plt.grid()
plt.show()
