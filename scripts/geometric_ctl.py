#!/usr/bin/env python
import math

import rospy
#from pid import PID
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from tf import transformations
import numpy as np
from dynamic_reconfigure.server import  Server
#from med_uav_control.cfg import MavZCtlParamsConfig
#from med_uav_control.msg import PIDController
from mav_msgs.msg import Actuators
from sim_controller import UAV
from scipy.integrate import ode
from scipy.integrate import odeint


'''

ATTITUDE CONTROL

Subscribes to:
        odometry           - used to extract attitude and attitude rate of the vehicle
        mot_vel_ref        - used to receive reference motor velocity from the height controller
        euler_ref          - used to set the attitude reference (useful for testing controllers) from horizontal controller
    
    Publishes:
        /gazebo/command/motor_speed     - reference motor velocities sent to each motor controller
        pid_roll           - publishes PID-roll data - reference value, measured value, P, I, D and total component (useful for tuning params)
        pid_roll_rate      - publishes PID-roll_rate data - reference value, measured value, P, I, D and total component (useful for tuning params)
        pid_pitch          - publishes PID-pitch data - reference value, measured value, P, I, D and total component (useful for tuning params)
        pid_pitch_rate     - publishes PID-pitch_rate data - reference value, measured value, P, I, D and total component (useful for tuning params)
        pid_yaw            - publishes PID-yaw data - reference value, measured value, P, I, D and total component (useful for tuning params)
        pid_yaw_rate       - publishes PID-yaw_rate data - reference value, measured value, P, I, D and total component (useful for tuning params)

HORIZONTAL CONTROL

Subscribes to:
        odometry       - used to extract x and y position of the vehicle
        /bebop/vel_ref   - used to extract vx and vy of the vehicle
        pos_ref    - used to set the reference for x and y -position
        ne postoji - vel_ref    - used to set the reference for vx and vy-position (useful for testing velocity controller)

    Publishes:
        euler_ref  - publishes referent values for euler angles (roll, pitch, yaw)
        pid_x      - publishes PID-x data - referent value, measured value, P, I, D and total component (useful for tuning params)
        pid_vx     - publishes PID-vx data - referent value, measured value, P, I, D and total component (useful for tuning params)
        pid_y      - publishes PID-y data - referent value, measured value, P, I, D and total component (useful for tuning params)
        pid_vy     - publishes PID-vy data - referent value, measured value, P, I, D and total component (useful for tuning params)

HEIGHT CONTROL

Subscribes to:
        odometry    - used to extract z-position and velocity of the vehicle
        pos_ref    - used to set the reference for z-position
        /bebop/vel_ref    - used to set the reference for vz-position (useful for testing velocity controller)

    Publishes:
        mot_vel_ref  - referent value for thrust in terms of motor velocity (rad/s)
        pid_z        - publishes PID-z data - referent value, measured value, P, I, D and total component (useful for tuning params)
        pid_vz        - publishes PID-vz data - referent value, measured value, P, I, D and total component (useful for tuning params)


'''

class GeometricController:

    def __init__(self):

        self.mot_speed = 0
        self.count = 0

        # initialize values:
        self.odometry = None
        self.odometry_gt = None
        self.pos_ref = None

        # initialize position in world frame:
        self.pos_x = 0
        self.pos_y = 0
        self.pos_z = 0

        # linearne brzine:
        self.linear_x = 0
        self.linear_y = 0
        self.linear_z = 0

        # angular velocities rates:
        self.euler_rate_x = 0
        self.euler_rate_y = 0
        self.euler_rate_z = 0

        # initialize euler angles:
        self.euler_x = 0
        self.euler_y = 0
        self.euler_z = 0

        # rotation matrix:
        self.R = [[0,0,0], [0,0,0], [0,0,0]]

        # state vector:
        self.X = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

        # initialize subscribers/publishers:
        rospy.Subscriber('odometry', Odometry, self.odometry_cb)
        rospy.Subscriber('pos_ref', Vector3, self.odometry_cb)
        rospy.Subscriber('odometry_gt', Odometry, self.odometry_gt_cb)
        self.mot_ref_pub = rospy.Publisher('mot_vel_ref', Float32, queue_size=1)
        self.pub_mot = rospy.Publisher('/gazebo/command/motor_speed', Actuators, queue_size=1)

        self.ros_rate = rospy.Rate(100) # geometric control at 100 Hz

        # initialize UAV class
        e3 = np.array([0.,0.,1.])
        J = np.diag([0.00389, 0.00389, 0.0078]) # podaci uzeti sa stranice: https://github.com/larics/med_uav_description/blob/master/config/mathematical_model.pdf
        self.uav = UAV(J, e3)
    
    def odometry_gt_cb(self, msg):

        self.odometry_gt = msg

        # get position:
        self.pos_x = msg.pose.pose.position.x
        self.pos_y = msg.pose.pose.position.y
        self.pos_z = msg.pose.pose.position.z

        # get linear velocities:
        self.linear_x = msg.twist.twist.linear.x
        self.linear_y = msg.twist.twist.linear.y
        self.linear_z = msg.twist.twist.linear.z

        # get angular velocities:
        # self.ang_x = msg.twist.twist.angular.x
        # self.ang_y = msg.twist.twist.angular.y
        # self.ang_z = msg.twist.twist.angular.z

        # orjentacija:
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        # konverzija kvaterniona u eulerove kuteve (yaw - pitch - roll)
        self.euler_x = math.atan2(2 * (qw * qx + qy * qz), qw * qw - qx * qx - qy * qy + qz * qz)
        self.euler_y = math.asin(2 * (qw * qy - qx * qz))
        self.euler_z = math.atan2(2 * (qw * qz + qx * qy), qw * qw + qx * qx - qy * qy - qz * qz)

        # angular rate:
        p = msg.twist.twist.angular.x
        q = msg.twist.twist.angular.y
        r = msg.twist.twist.angular.z
        sx = math.sin(self.euler_x)   # sin(roll)
        cx = math.cos(self.euler_x)   # cos(roll)
        cy = math.cos(self.euler_y)   # cos(pitch)
        ty = math.tan(self.euler_y)   # tan(pitch)
        # conversion gyro measurements to roll_rate, pitch_rate, yaw_rate
        self.euler_rate_x = p + sx * ty * q + cx * ty * r
        self.euler_rate_y = cx * q - sx * r
        self.euler_rate_z = sx / cy * q + cx / cy * r

        # calculate R matrix:
        # redci predstavljaju pojedinu transformiranu os koordinatnog sustava.
        # redak 1. rotacijske matrice (x):
        R_x = [1-(2*pow(qy, 2))-(2*pow(qz, 2)), (2*qx*qy)-2*qz*qw, 2*qx*qz+2*qy*qw]
        # redak 2. rotacijske matrice (y):
        R_y = [(2*qx*qy)+(2*qz*qw), 1-(2*pow(qx, 2))-(2*pow(qz, 2)), (2*qy*qz)-(2*qx*qw)]
        # redak 3. rotacijske matrice (z):
        R_z = [(2*qx*qz)-(2*qy*qw), (2*qy*qz)+(2*qx*qw), 1-(2*pow(qx, 2))-(2*pow(qy, 2))]

        self.R.append(R_x)
        self.R.append(R_y)
        self.R.append(R_z)


    def odometry_cb(self, odom):
        # pohraniti u odometriju
        self.odometry = odom
        # print("ODOMETRIJA:", odom)

    def pos_ref_cb(self, pos):
        self.pos_ref = pos

    def run(self):
        
        while not rospy.is_shutdown():
            self.ros_rate.sleep()

            # inicijalizirati vektor stanja X iz proracunatih odometrijskih podataka:
            self.X = [self.pos_x, self.pos_y, self.pos_z, self.linear_x, self.linear_y, self.linear_z]
            self.X.extend(self.R[0])
            self.X.extend(self.R[1])
            self.X.extend(self.R[2])
            self.X.extend([self.euler_rate_x, self.euler_rate_y, self.euler_rate_z])

            # pozivati dydt


            # R0 = np.eye(3)
            # W0 = [0.,0.,0.];   # initial angular velocity
            # x0 = [0.,0.,0.];  # initial position (altitude?0)
            # v0 = [0.,0.,0.];   # initial velocity
            # R0v = np.array(R0).flatten().T
            # y0 = np.concatenate((x0, v0, R0v, W0))

            # dt = 1./100
            # sim = []
            # xd = []
            # xd_dot = []
            # command_val = []
            # solver = ode(self.uav.dydt)
            # solver.set_integrator('dopri5').set_initial_value(y0, 0)

            # solver.integrate(solver.t+dt)
            # sim.append(solver.y)
            # xd.append(self.uav.xd)
            # xd_dot.append(self.uav.xd_dot)
            # command_val.append(self.uav.command)

            print('pozivam funkciju dydt!')
            command_val = self.uav.dydt(self.count+0.01, self.X)

            print('COMMAND VAL iz dydt:')
            print(self.uav.command)

            mot_speed_msg = Actuators()
            mot_speed_msg.angular_velocities = [self.uav.command[0],self.uav.command[1],self.uav.command[2],self.uav.command[3]]
            
            
            #mot_speed_msg.angular_velocities = [command_val[0],command_val[1],command_val[0][2],command_val[0][3]]            
            
            print('MOTOR SPEED MESSAGE:')
            print(mot_speed_msg)
            self.pub_mot.publish(mot_speed_msg)

            # pomocu f, M dobivenih od dydt upravljati letjelicom (pretvoriti u brzine vrtnje):

            #mot_sp1 = self.w_sp - dw_roll - dw_pitch - dw_yaw # reference value for motor velocity w.sp
            #mot_sp2 = self.w_sp + dw_roll - dw_pitch + dw_yaw
            #mot_sp3 = self.w_sp + dw_roll + dw_pitch - dw_yaw
            #mot_sp4 = self.w_sp - dw_roll + dw_pitch + dw_yaw
        
            # mot_speed_msg = Actuators()
            # mot_speed_msg.angular_velocities = [self.mot_speed,self.mot_speed,self.mot_speed, self.mot_speed]
            # self.pub_mot.publish(mot_speed_msg)

if __name__ == '__main__':

    rospy.init_node('geometric_controller')
    geom_ctl = GeometricController()
    geom_ctl.run()