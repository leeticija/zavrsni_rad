#!/usr/bin/env python
from __future__ import print_function, division, absolute_import
import numpy as np
from scipy.integrate import odeint
from scipy.integrate import ode
import numpy.linalg as la
import pdb
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
import matplotlib.animation as animation
import sys
import ukf_uav


class UAV(object):

    def __init__(self, J, e3):
        self.m = 0.5
        self.bf = 0.0000854858 # thrust constant of a motor.
        self.ctf = 0.016 # moment constant of a motor.
        self.d = 0.12905 # distance from center of mass to motor.
        self.g = 9.81
        self.J = J
        self.e3 = e3
        self.kR = 8.81 # attitude gains
        self.kW = 2.54 # angular rate gain
        #self.kx = 20*self.m # position gains
        #self.kv = 195.9*self.m # velocity gains
        self.kx = 1
        self.kv = 1
        self.xd = None
        self.xd_dot = None
        self.command = [0,0,0,0]
        print('UAV: initialized')


    # xd, xd_dot i xd_ddot paramere bi trebao davati planer.
    # xd, xd_dot, xd_ddot
    def calculate_commands(self, t, X, xd, xd_dot, xd_ddot, pos_ref):

        # toppra planer? ako cemo ga koristiti - u trenutku t na kojoj poziciji, koju brzinu i akceleraciju mora imati letjelica.
        # -> to su desired vrijednosti i njih pretpostavljam u ovoj implementaciji.

        R = np.reshape(X[6:15],(3,3));  # rotation from body to inertial
        W = X[15:];   # current angular rate
        x = X[:3];  # current position
        v = X[3:6];    # current velocity

        #xd = np.array([0, 0, 0])
        #xd_dot = np.array([0, 0, 0])
        #xd_ddot = np.array([0, 0, 0])
        xd_dddot = np.array([0, 0, 0])
        xd_ddddot = np.array([0, 0, 0])
        b1d = np.array([1., 0., 0.])
        b1d_dot=np.array([0., 0., 0.])
        b1d_ddot=np.array([0., 0., 0.])
        Rd = np.eye(3)
        Wd = np.array([0.,0.,0.])
        Wd_dot = np.array([0.,0.,0.])
        f = np.array([0,0,0])
        M = np.array([0,0,0])

        # desired values:
        Rd = R
        Rd_dot = np.array([[0,0,0],[0,0,0],[0,0,0]])
        #Rd = np.array([[np.cos(ang_d), 0., np.sin(ang_d)],[0.,1.,0.],
        #        [-np.sin(ang_d), 0., np.cos(ang_d)]])
        #Rd_dot = np.array([[-ang_d_dot*np.sin(ang_d), 0.,
        #        ang_d_dot*np.cos(ang_d)],[0.,0.,0.],
        #        [-ang_d_dot*np.cos(ang_d), 0., -ang_d_dot*np.sin(ang_d)]])
        
        # toppra planer daje desired vrijednosti za brzinu i akceleraciju, ne znam sto da stavim za desired kutne brzine?:
        Wdhat=Rd.T.dot(Rd_dot)
        Wd=np.array([-Wdhat[1,2],Wdhat[0,2],-Wdhat[0,1]])

        d_in = (xd, xd_dot, xd_ddot, xd_dddot, xd_ddddot,
                b1d, b1d_dot, b1d_ddot, Rd, Wd, Wd_dot)
        (f, M) = self.position_control(t, R, W, x, v, d_in)

        R_dot = np.dot(R,hat(W))
        W_dot = np.dot(la.inv(self.J), M - np.cross(W, np.dot(self.J, W)))
        x_dot = v
        v_dot = self.g*self.e3 - f*R.dot(self.e3)/self.m
        X_dot = np.concatenate((x_dot, v_dot, R_dot.flatten(), W_dot))
        self.xd = xd
        self.xd_dot = xd_dot
        self.command = np.insert(M,0,f)
    

    def attitude_control(self, t, R, W, x, v, d_in):
        (xd, xd_dot, xd_ddot, xd_dddot, xd_ddddot, b1d, b1d_dot, b1d_ddot, Rd, Wd, Wd_dot) = d_in
        (ex, ev) = position_errors( x, xd, v, xd_dot)
        f = (self.kx*ex + self.kv*v + self.m*self.g*self.e3).dot(R.dot(self.e3))
        W_hat = hat(W)
        (eR, eW) = attitude_errors( R, Rd, W, Wd )
        M= -self.kR*eR - self.kW*eW + np.cross(W, self.J.dot(W)) - self.J.dot(W_hat.dot(R.T.dot(Rd.dot(Wd))) - R.T.dot(Rd.dot(Wd_dot)))
        return (f, M)


    def position_control(self, t, R, W, x, v, d_in):
        (xd, xd_dot, xd_2dot, xd_3dot, xd_4dot, b1d, b1d_dot, b1d_ddot, Rd, Wd, Wd_dot) = d_in
        (ex, ev) = position_errors( x, xd, v, xd_dot)

        f = np.dot(self.kx*ex + self.kv*ev + self.m*self.g*self.e3 - self.m*xd_2dot, R.dot(self.e3) )
        W_hat = hat(W)
        R_dot = R.dot(W_hat)
        x_2dot = self.g*self.e3 - f*R.dot(self.e3)/self.m
        ex_2dot = x_2dot - xd_2dot

        f_dot = ( self.kx*ev + self.kv*ex_2dot - self.m*xd_3dot).dot(R.dot(self.e3)) + ( self.kx*ex + self.kv*ev + self.m*self.g*self.e3 - self.m*xd_3dot).dot(np.dot(R_dot,self.e3))

        x_3dot = -1/self.m*( f_dot*R + f*R_dot ).dot(self.e3)
        ex_3dot = x_3dot - xd_3dot

        A = -self.kx*ex - self.kv*ev - self.m*self.g*self.e3 + self.m*xd_2dot
        A_dot = -self.kx*ev - self.kv*ex_2dot + self.m*xd_3dot
        A_2dot = -self.kx*ex_2dot - self.kv*ex_3dot + self.m*xd_4dot

        (Rd, Wd, Wd_dot) = get_Rc(A, A_dot, A_2dot , b1d, b1d_dot, b1d_ddot)

        (eR, eW) = attitude_errors( R, Rd, W, Wd )
        M= -self.kR*eR - self.kW*eW + np.cross(W, self.J.dot(W)) - self.J.dot(W_hat.dot(R.T.dot(Rd.dot(Wd))) - R.T.dot(Rd.dot(Wd_dot)))
        return (f, M)

    def vee(self, M):
        return np.array([M[2,1], M[0,2], M[1,0]])
    
    def hat(self, x):
        hat_x = [0, -x[2], x[1],
                x[2], 0, -x[0],
                -x[1], x[0], 0]
        return np.reshape(hat_x,(3,3))

    def rot_eul(self, x_in):
        theta_x = np.arctan2(x_in[:,7], x_in[:,8])
        theta_y = np.arctan2(x_in[:,6], (x_in[:,7]**2+x_in[:,8]**2)**(1/2))
        theta_z = np.arctan2(x_in[:,1], x_in[:,0])
        return np.array([theta_x,theta_y,theta_z]).T

    # pos_ref je array s brojevima - referentna pozicija
    def get_commands(self, t, X, pos_ref):
        # f = - (-kx*ex - kv*ev - m*g*e3 + m*xd_ddot) puta R*e3
        # M = -kR*eR - kΩ*eΩ + Ω x J*Ω - J(Ωkappa * Rtrans * Rd * Ωd - Rtrans * Rd * Ωd_dot)

        # xd je referentna trazena pozicija (tocka u prostoru u koord. sustavu svijeta koju je potrebno postici u trenutku t).
        xd = np.array([0,0,1])   
        # derivacije od xd:

        # 1. ex = x - xd ----> position error u trenutku t:
        x = X[:3] # current position in world frame (vektor od 3 vrijednosti)
        x = np.array(x)
        ex = x - xd # vektor pogreske pozicije

        print('ex:')
        print(ex)

        #xd_dot = np.array([0,0,(1-x[2])/10])
        xd_dot = [0,0,0]
        xd_ddot = np.array([0,0,0])

        # 2. ev = v - vd ----> velocity error u trenutku t:
        v = np.array(X[3:6])
        vd = xd_dot # referentna brzina
        ev = v - vd

        # 3. R ----> rotacijska matrica
        R = np.array([X[6:9], X[9:12], X[12:15]])  # rotation from body fixed frame to inertial frame.
        
        #print('umnozak:', R*self.e3)
        # total thrust f = - (-kx*ex - kv*ev - m*g*e3 + m*xd_ddot) puta R*e3
        m1 = -(-self.kx*ex - self.kv*ev - self.m*self.g*self.e3 + self.m*xd_ddot)
        m2 = R*self.e3
        f = np.dot(m1, m2)

        print('TOTAl THRUST:')
        print(f)

        # izracunati M

        b3d = np.array([0, 0, -1]) # -1 je jer taj vektor pokazuje prema dolje, suprotno od ukupne sile thrusta.
        b2d = np.array([0, 1, 0])
        # b1d = np.array([-1,0,0]) to je zapravo cross product od b2d i b3d, ali pitanje je je li desni ili ljevi koordinatni sustav?
        
        # M = -kR*eR - kΩ*eΩ + Ω x J*Ω - J(Ωkappa * Rtrans * Rd * Ωd - Rtras * Rd * Ωd_dot)

        # 1. eR = 1/2(Rd trans * R - R trans * Rd) vee
        
        # Rd = [b2d x b3d, b2d, b3d]
        print('b1d:', np.cross(b2d, b3d))
        #Rd = np.array([np.cross(b2d, b3d), b2d, b3d])
        Rd = np.array([[0,0,0], [0,1,0], [0,0,-1]])
        print('Rd:')
        print(Rd)

        Rd_trans = Rd.transpose()
        R_trans = R.transpose()
        diff = np.matmul(Rd_trans, R) - (np.matmul(R_trans, Rd))
        vee = self.vee(diff)
        eR = (1/2) * vee
        print('eR:')
        print(eR)

        # 2. eΩ = Ω -R trans * Rd * Ωd

        # Ω = W
        W = np.array(X[15:])   # angular rate
        Wd = np.array([0,0,0]) # je li desired angular velocity 0?
        Wd_dot = np.array([0,0,0])
        multiplied = np.matmul(R_trans, Rd)

        eW = W - (np.matmul(multiplied, Wd))

        print('eW:')
        print(eW)

        # 3. izracunati M!
        cross_product = np.cross(W, np.matmul(self.J, W))
        print('cross product:')
        print(cross_product)
        multiplied = self.J*((self.hat(W)*np.matmul(R_trans, Rd))*Wd - (np.matmul(R_trans, Rd)*Wd_dot))
        print('multiplied:')
        print(multiplied)
        M = - self.kR*eR - self.kW*eW + cross_product - multiplied

        M = M[0] # jer su svi retci isti, a trebam jednodimenzionalnu.
        print("M:")
        print(M)

        self.command = np.insert(M, 0, f[2])

        print('COMMAND:')
        print(self.command)

        return self.command

def vee(M):
    return np.array([M[2,1], M[0,2], M[1,0]])

def attitude_errors( R, Rd, W, Wd ):
    eR = 0.5*vee(Rd.T.dot(R) - R.T.dot(Rd))
    eW = W - R.T.dot(Rd.dot(Wd))
    return (eR, eW)

def position_errors(x, xd, v, vd):
    ex = x - xd
    ev = v - vd
    return (ex, ev)

def hat(x):
    hat_x = [0, -x[2], x[1],
            x[2], 0, -x[0],
            -x[1], x[0], 0]
    return np.reshape(hat_x,(3,3))

def get_Rc(A, A_dot, A_2dot, b1d, b1d_dot, b1d_ddot):
    # move this as a function
    norm_A = la.norm(A)
    b3c = - A/norm_A
    b3c_dot = - A_dot/norm_A + ( np.dot(A, A_dot)*A )/norm_A**3
    b3c_2dot = - A_2dot/norm_A + ( 2*np.dot(A*A_dot,A_dot) )/norm_A**3 + np.dot( A_dot* A_dot + A*A_2dot ,A)/norm_A**3 - 3*np.dot((A*A_dot)**2,A)/norm_A**5

    b_ = np.cross(b3c, b1d)
    b_norm = la.norm(b_)
    b_dot = np.cross(b3c_dot, b1d) + np.cross(b3c, b1d_dot)
    b_2dot = np.cross(b3c_2dot, b1d) + 2*np.cross(b3c_dot, b1d_dot) + np.cross(b3c, b1d_ddot)

    b1c = -np.cross( b3c, b_ )/b_norm
    b1c_dot = -( np.cross(b3c_dot, b_) + np.cross(b3c, b_dot) )/b_norm + np.cross(b3c, b_)*(b_dot* b_)/b_norm**3

    # intermediate steps to calculate b1c_2dot
    m_1 = ( np.cross(b3c_2dot, b_) + 2*np.cross(b3c_dot, b_dot) + np.cross(b3c, b_2dot) )/b_norm
    m_2 = ( np.cross(b3c_dot, b_) + np.cross(b3c, b_dot) )*np.dot(b_dot, b_)/b_norm**3
    m_dot = m_1 - m_2
    n_1 = np.cross(b3c, b_)*np.dot(b_dot, b_)
    n_1dot = ( np.cross(b3c_dot, b_) + np.cross(b3c, b_dot) )*np.dot(b_dot, b_) + np.cross(b3c, b_)*( np.dot(b_2dot, b_)+np.dot(b_dot, b_dot) )
    n_dot = n_1dot/b_norm**3 - 3*n_1*np.dot(b_dot, b_)/b_norm**5
    b1c_2dot = -m_dot + n_dot

    Rc = np.reshape([b1c, np.cross(b3c, b1c), b3c],(3,3)).T
    Rc_dot = np.reshape([b1c_dot, ( np.cross(b3c_dot, b1c) + np.cross(b3c, b1c_dot) ), b3c_dot],(3,3)).T
    Rc_2dot = np.reshape( [b1c_2dot, ( np.cross(b3c_2dot, b1c) + np.cross(b3c_dot, b1c_dot) + np.cross(b3c_dot, b1c_dot) + np.cross(b3c, b1c_2dot) ), b3c_2dot],(3,3)).T
    Wc = vee(Rc.T.dot(Rc_dot))
    Wc_dot= vee( Rc_dot.T.dot(Rc_dot) + Rc.T.dot(Rc_2dot))
    return (Rc, Wc, Wc_dot)