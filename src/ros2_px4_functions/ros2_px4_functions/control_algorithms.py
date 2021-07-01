import numpy as np
import scipy.linalg
from scipy.interpolate import interp1d

class PID_controller:
    def __init__(self, kp_, ki_, kd_, u_max_, u_min_, int_max_, dt_):

        self.kp_ = kp_
        self.ki_ = ki_
        self.kd_ = kd_
        self.u_max_ = u_max_
        self.u_min_ = u_min_
        self.int_max_ = int_max_
        self.dt_ = dt_

        self.e_old_ = []
        self.e_int_ = 0.0

    def PID(self, e_):

        if self.e_old_!=[]:
            self.e_dot_ = (e_ - self.e_old_)/self.dt_
        else:
            self.e_dot_ = np.zeros(len(e_))

        self.e_int_ = self.e_int_ + e_
        self.e_int_ = np.clip(self.e_int_, - self.int_max_, self.int_max_)

        self.e_old_ = e_
        uk_ = - np.multiply(self.kp_, e_) - np.multiply(self.ki_, self.e_int_) - np.multiply(self.kd_, self.e_dot_)

        uk_ = np.clip(uk_, self.u_min_, self.u_max_)

        return uk_, self.e_dot_


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
    return K


def DLQR(K, xk, u_max, u_min):

    uk = K*xk
    uk = np.clip(uk, u_min, u_max)

    return uk


def gain_scheduling_optimizer(list_states, list_kp, list_ki, list_kd):

    kp_interpolated = interp1d(list_states, list_kp, kind='linear')
    ki_interpolated = interp1d(list_states, list_ki, kind='linear')
    kd_interpolated = interp1d(list_states, list_kd, kind='linear')

    return kp_interpolated, ki_interpolated, kd_interpolated
