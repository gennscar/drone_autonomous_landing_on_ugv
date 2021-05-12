import functions
import numpy as np
import matplotlib.pyplot as plt
from scipy import signal

# Simulation parameters
dt = 0.1
x0 = np.matrix("1; 1")
ref = [0, 0]
t_sim = 60
time = np.linspace(0, t_sim, t_sim/dt, endpoint=True)

# System
A = np.matrix([[0, 0], [0, 0]])
B = np.matrix([[1, 0], [0, 1]])
C = np.matrix([[1, 0], [0, 1]])
D = np.zeros((2, 2))
sys = signal.StateSpace(A, B, C, D)
sys = sys.to_discrete(dt)
Ad = sys.A
Bd = sys.B
Cd = sys.C
Dd = sys.D

# LQR
Q = np.matrix([[1, 0], [0, 1]])*1
R = np.matrix([[1, 0], [0, 1]])*4
K = functions.DLQR_optimizer(Ad, Bd, Q, R)

G_i = 0.0000001
G_xy = 10
G_r = 40

Q_aug = np.matrix([[G_i, 0, 0, 0], [0, G_i, 0, 0], [
                  0, 0, G_xy, 0], [0, 0, 0, G_xy]])*1
R_aug = np.matrix([[1, 0], [0, 1]])*G_r
A_aug = np.r_[np.c_[[[1, 0], [0, 1]], -dt*Cd], np.c_[np.zeros((2, 2)), Ad]]
B_aug = np.r_[np.zeros((2, 2)), Bd]
K_aug = functions.DLQR_optimizer(A_aug, B_aug, Q_aug, R_aug)
Ki = K_aug[:, 0:2]
Ko = K_aug[:, 2:]
print(Ki)
print(Ko)

# PID
kp = 0.5
ki = 0.0001
kd = 0.0001
e_old = np.array([0, 0])
int_e = np.array([0, 0])
u_max = 10  # float("inf")
u_min = - u_max
int_max = float("inf")

# Simulation LQR
xk = x0
t = 0
e = np.zeros((2, 1))
e_int = np.zeros((2, 1))
EX_LQR = []
EY_LQR = []
UX_LQR = []
UY_LQR = []

while t < t_sim:
    # LQR
    # uk = ref - K*xk

    # LQR + INTEGRATOR
    uk = - Ki*e_int - Ko*xk
    uk = np.clip(uk, -u_max, u_max)
    xk = Ad*xk + Bd*uk
    e = np.subtract(ref, xk)
    e_int = e_int + e

    EX_LQR.append(e[0, 0])
    EY_LQR.append(e[1, 0])
    UX_LQR.append(uk[0, 0])
    UY_LQR.append(uk[1, 0])

    t += dt

# Simulation PID
xk = x0
t = 0
e = np.zeros((2, 1))
e_int = np.zeros((2, 1))
EX_PID = []
EY_PID = []
UX_PID = []
UY_PID = []

while t < t_sim:

    # PID
    uk, int_e, e_dot, e_old = functions.PID(
        kp, ki, kd, xk, e_old, int_e, u_max, u_min, int_max)

    xk = Ad*xk + Bd*uk
    e = np.subtract(ref, xk)
    e_int = e_int + e

    EX_PID.append(e[0, 0])
    EY_PID.append(e[1, 0])
    UX_PID.append(uk[0, 0])
    UY_PID.append(uk[1, 0])

    t += dt

# Plots
fig, axs = plt.subplots(2, sharex=True)
fig.suptitle('Error and velocity')
axs[0].plot(time, EX_LQR, label="EX_LQR error, meters")
axs[0].plot(time, EY_LQR, label="EY_LQR error, meters")
axs[0].plot(time, EX_PID, label="EX_PID error, meters")
axs[0].plot(time, EY_PID, label="EY_PID error, meters")
axs[0].legend(loc='best')
axs[0].grid()
axs[1].plot(time, UX_LQR, label="X velocity LQR, meters")
axs[1].plot(time, UY_LQR, label="Y velocity LQR, meters")
axs[1].plot(time, UX_PID, label="X velocity PID, meters")
axs[1].plot(time, UY_PID, label="Y velocity PID, meters")
axs[1].legend(loc='best')
axs[1].grid()
plt.show()
