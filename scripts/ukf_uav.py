from __future__ import (absolute_import, division, print_function, unicode_literals)
import numpy as np
# from numba import jit, int16
from numpy.linalg import cholesky, inv, det
import matplotlib.pyplot as plt
import pdb


class UnscentedKalmanFilter(object):
    """Filtering implementation for state estimation
    This module estimates the state of the UAV from given sensor meaurements
    using unscented Kalman filter.
    state:
        x: position, velocity
        R: attitude, angular velocity
        power: current, voltage, RPM
    state vector:
        state: [x,x_dot,R,W]
    """
    def __init__(self, dim_x=1, dim_z=1, dt=0.1, hx=None, fx=None):
        self._dt = dt
        self._dim_x = dim_x
        self._dim_z = dim_z
        self.hx = hx
        self.fx = fx
        self.x = np.zeros(dim_x)
        self.P = np.eye(dim_x)
        self.Q = np.eye(dim_x)
        self.R = np.eye(dim_z)
        self.m = 1.5
        self.g = 9.8
        self.J = np.eye(3)
        self.R = np.eye(3)
        self.e3 = np.array([0,0,1])
        print('Unscented Kalman Filter initialized')

    def sigmaPoints(self, x, P, c):
        """Simg points computation"""
        A = c*cholesky(P).T
        Y = np.tile(x,((A.shape)[0],1)).T
        return np.vstack((x, Y+A, Y-A)).T

    def dss(self, x, u, dt=None):
        if dt == None:
            dt = self._dt
        """State space UAV dynamics"""
        A = np.zeros((12,12))
        for i in range(6):
            A[i][i], A[i][i+1] = 1, dt
            A[i+6][i+6] = 1
        # TODO: Implement controller input term here
        # noting that u = [F, M] remember to bring them to inertial frame
        B = np.zeros((self._dim_x, 6))
        B[3:6,0:3] = 1/self.m*np.eye(3)
        B[-3:,-3:] = self.J
        #uf = -u[0]*self.R.dot(self.e3)
        uf = -u[0]*self.e3.dot(self.R)
        #uf = np.append(uf, self.R.dot(u[1:]))
        uf = np.append(uf, u[1:].dot(self.R))
        # print(x)
        # print(self._dt*B.dot(uf))
        xp = x + self._dt*B.dot(uf)
        print("xp:", xp)
        return xp

    def sss(self, x, u = None):
        """Sensor state"""
        A = np.zeros((6,12))
        for i in range(6):
            A[i][i+6] = 1
        return A.dot(x)

    def f(x):
        """nonlinear state function"""
        return np.array([x[1],x[2],0.05*x[0]*(x[1]+x[2])])

    def h(x):
        """nonlinear sensor function"""
        return np.array([x[0]])

    def ut(self, func, x, wm, wc, n_f, Q, u = None):
        """unscented transform
        args:
            f: nonlinear map
        output:
            mean
            sampling points
            covariance
            deviations
        """
        n, m = x.shape
        X = np.zeros((n_f,m))
        mu = np.zeros(n_f)
        for i in range(m):
            X[:,i] = func(x[:,i], u)
            mu = mu + wm[i]*X[:,i]
        Xd = X - np.tile(mu,(m,1)).T
        Sigma = Xd.dot(np.diag(wc.T).dot(Xd.T)) + Q
        return (mu, X, Sigma, Xd)

    def ukf(self, x, P, z, Q, R, u):
        """UKF
        args:
            x: a priori state estimate
            P: a priori state covariance
            z: current measurement
            Q: process noise
            R: measurement noise
        output:
            x: a posteriori state estimation
            P: a posteriori state covariance
        """
        n = len(x)
        m = len(x)
        alpha = 0.75
        kappa = 0.
        beta = 2.
        lamb = alpha**2*(n+kappa)-n
        c_n = n+lamb
        Wm = np.append(lamb/c_n, 0.5/c_n+np.zeros(2*n))
        Wc = np.copy(Wm)
        Wc[0] +=  (1-alpha**2+beta)
        c_nr=np.sqrt(c_n)
        X = self.sigmaPoints(x,P,c_nr)
        x1, X1, P1, X2 = self.ut(self.dss, X, Wm, Wc, n, Q, u)
        z1,Z1,P2,Z2 = self.ut(self.sss, X1,Wm,Wc, int(n/2), R)
        P12=X2.dot(np.diag(Wc).dot(Z2.T))
        K=P12.dot(inv(P2))
        x=x1+K.dot(z-z1)
        P=P1-K.dot(P12.T)
        return x, P

    # @jit(int16(int16))
#    def test(self, a = 3):
#        x = a
#        return x

if __name__=='__main__':
    # test(4)
    # test.inspect_types()
    # print('test')
    ukf_t = UnscentedKalmanFilter(12, 6, 0.01)
    # ukf_t.ukf(x,P, z, Q, R, u)
    Ns = 12 # number of states
    s = np.zeros(Ns)
    u = np.zeros(4)
    q=0.1
    r=0.1
    Q = q**2*np.eye(Ns)
    R = r**2
    P = np.eye(Ns)
    x = s+q*np.random.random(Ns)
    N = 20
    xV = np.zeros((Ns,N))
    sV = np.copy(xV)
    zV = np.zeros((6,N))
    for k in range(N):
        z = ukf_t.sss(s)+r*np.random.random()
        sV[:,k] = s
        zV[:,k] = z
        x, P = ukf_t.ukf(x, P, z, Q, R, u)
        xV[:,k] = x
        s = ukf_t.dss(s,u) + q*np.random.random(Ns)
        pass