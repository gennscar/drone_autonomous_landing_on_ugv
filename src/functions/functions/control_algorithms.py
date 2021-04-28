import numpy as np
import scipy.linalg
from scipy.interpolate import interp1d

def PID(kp, ki, kd, e, e_old, int_e, u_max, u_min, int_max):

    int_e = int_e + e
    e_dot = (e - e_old)/0.1
    e_old = e
    uk = - np.multiply(kp, e) - np.multiply(ki, int_e) - np.multiply(kd, e_dot)
    
    uk = np.clip(uk, u_min, u_max)
    int_e = np.clip(int_e, - int_max, int_max)
    
    return uk, int_e, e_dot, e_old

def DLQR_optimizer(A, B, Q, R):

    """
    Solve the discrete time lqr controller.
    x[k+1] = A x[k] + B u[k]
    cost = sum x[k].T*Q*x[k] + u[k].T*R*u[k]
    """
    # first, solve the riccati equation
    P = np.matrix(scipy.linalg.solve_discrete_are(A, B, Q, R))
    # compute the LQR gain
    K = np.matrix(scipy.linalg.inv(B.T*P*B+R)*(B.T*P*A))
    return -K

def DLQR(K, xk, u_max, u_min):

    uk = K*xk
    uk = np.clip(uk, u_min, u_max)


    return uk
    
def gain_scheduling_optimizer(list_states, list_kp, list_ki, list_kd):

    kp_interpolated = interp1d(list_states, list_kp, kind='linear')
    ki_interpolated = interp1d(list_states, list_ki, kind='linear')
    kd_interpolated = interp1d(list_states, list_kd, kind='linear')

    return kp_interpolated, ki_interpolated, kd_interpolated




