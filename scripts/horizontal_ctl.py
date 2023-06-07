#!/usr/bin/env python

__author__ = 'thaus'

import rospy
from pid import PID
from geometry_msgs.msg import Vector3, Vector3Stamped, PoseWithCovarianceStamped, TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from dynamic_reconfigure.server import Server
from med_uav_control.cfg import MavXYCtlParamsConfig
from med_uav_control.msg import PIDController
from tf import transformations
import numpy
import math

class HorizontalControl:
    '''
    Class implements ROS node for cascade (z, vz) PID control for MAV height.
    Subscribes to:
        pose       - used to extract x any y position of the vehicle
        euler      - used to extract measured yaw
        velocity   - used to extract vx and vy of the vehicle
        pos_ref    - used to set the reference for x and y -position
        vel_ref    - used to set the reference for vx and vy-position (useful for testing velocity controller)

    Publishes:
        euler_ref  - publishes referent values for euler angles (roll, pitch, yaw)
        pid_x      - publishes PID-x data - referent value, measured value, P, I, D and total component (useful for tuning params)
        pid_vx     - publishes PID-vx data - referent value, measured value, P, I, D and total component (useful for tuning params)
        pid_y      - publishes PID-x data - referent value, measured value, P, I, D and total component (useful for tuning params)
        pid_vy     - publishes PID-vx data - referent value, measured value, P, I, D and total component (useful for tuning params)

    Dynamic reconfigure is used to set controller params online.
    '''

    def __init__(self):
        '''
        Initialization of the class.
        '''

        self.start_flag = False         # indicates if we received the first measurement
        self.config_start = False       # flag indicates if the config callback is called for the first time

        self.yaw_sp = 0                 # referent yaw value
        self.yaw_mv = 0                 # measured yaw value

        self.x_sp = 0                   # x-position set point
        self.x_mv = 0                   # x-position measured value
        self.pid_x = PID()              # pid instance for x control

        self.vx_sp = 0                  # vx velocity set_point
        self.vx_mv = 0                  # vx velocity measured value
        self.pid_vx = PID()             # pid instance for x-velocity control

        self.y_sp = 0                   # y-position set point
        self.y_mv = 0                   # y-position measured value
        self.pid_y = PID()              # pid instance for y control

        self.vy_sp = 0                  # vy velocity set_point
        self.vy_mv = 0                  # vy velocity measured value
        self.pid_vy = PID()             # pid instance for y-velocity control

        #########################################################
        #########################################################
        # Add parameters for x controller
        self.pid_x.set_kp(0.7)
        self.pid_x.set_ki(0)
        self.pid_x.set_kd(0)

        # Add parameters for vx controller
        self.pid_vx.set_kp(0.25)
        self.pid_vx.set_ki(0.03)
        self.pid_vx.set_kd(0.015)

        # Add parameters for y controller
        self.pid_y.set_kp(0.7)
        self.pid_y.set_ki(0)
        self.pid_y.set_kd(0)

        # Add parameters for vy controller
        self.pid_vy.set_kp(0.25)
        self.pid_vy.set_ki(0.03)
        self.pid_vy.set_kd(0.015)
        #########################################################
        #########################################################

        self.pid_x.set_lim_up(15)        # max vx speed
        self.pid_x.set_lim_low(-15)      # min vx speed

        self.pid_vx.set_lim_up(45.0/180 * math.pi)        # max pitch angle
        self.pid_vx.set_lim_low(-45.0/180 * math.pi)      # min pitch angle

        self.pid_y.set_lim_up(15)        # max vy speed
        self.pid_y.set_lim_low(-15)      # min vy speed

        self.pid_vy.set_lim_up(45.0/180 * math.pi)        # max roll angle
        self.pid_vy.set_lim_low(-45.0/180 * math.pi)      # min roll angle

        self.quaternion = numpy.empty((4, ), dtype=numpy.float64)
        self.velocity_b = numpy.empty((3, ), dtype=numpy.float64)
        self.velocity_i = numpy.empty((3, ), dtype=numpy.float64)

        rospy.Subscriber('odometry', Odometry, self.odometry_cb)
        rospy.Subscriber('vel_ref', Vector3, self.vel_ref_cb)
        rospy.Subscriber('pos_ref', Vector3, self.pos_ref_cb)
        rospy.Subscriber('yaw_ref', Float32, self.yaw_ref_cb)
        self.pub_pid_x = rospy.Publisher('pid_x', PIDController, queue_size=1)
        self.pub_pid_vx = rospy.Publisher('pid_vx', PIDController, queue_size=1)
        self.pub_pid_y = rospy.Publisher('pid_y', PIDController, queue_size=1)
        self.pub_pid_vy = rospy.Publisher('pid_vy', PIDController, queue_size=1)
        self.euler_ref_pub = rospy.Publisher('euler_ref', Vector3, queue_size=1)
        self.cfg_server = Server(MavXYCtlParamsConfig, self.cfg_callback)
        self.ros_rate = rospy.Rate(10)
        self.t_start = rospy.Time.now()

    def run(self):
        '''
        Runs ROS node - computes PID algorithms for z and vz control.
        '''

        while not self.start_flag:
            print('Waiting for velocity measurements.')
            rospy.sleep(0.5)
        print("Starting horizontal control.")

        while not rospy.is_shutdown():
            self.ros_rate.sleep()

            ########################################################
            ########################################################
            # Implement cascade PID control here.
            # Reference for x is stored in self.x_sp.
            # Measured x-position is stored in self.x_mv.
            # If you want to test only vx - controller, the corresponding reference is stored in self.vx_sp.
            # Measured vx-velocity is stored in self.vx_mv
            # Reference for y is stored in self.y_sp.
            # Measured y-position is stored in self.y_mv.
            # If you want to test only vy - controller, the corresponding reference is stored in self.vy_sp.
            # Measured vx-velocity is stored in self.vy_mv
            # Resultant referent value for roll and pitch (in mobile coordinate system!)
            # should be stored in variable roll_sp and pitch_sp

            vx_sp = self.pid_x.compute(self.x_sp, self.x_mv)
            pitch_sp_g = self.pid_vx.compute(vx_sp, self.vx_mv)  # this is in global frame

            vy_sp = self.pid_y.compute(self.y_sp, self.y_mv)
            roll_sp_g = -self.pid_vy.compute(vy_sp, self.vy_mv)  # this is in global frame

            # decouple with respect to the measured yaw
            pitch_sp = math.cos(self.yaw_mv) * pitch_sp_g - math.sin(self.yaw_mv) * roll_sp_g
            roll_sp = math.sin(self.yaw_mv) * pitch_sp_g + math.cos(self.yaw_mv) * roll_sp_g
            ########################################################
            ########################################################

            euler_sv = Vector3(roll_sp, pitch_sp, self.yaw_sp)
            self.euler_ref_pub.publish(euler_sv)

            # Publish PID data - could be usefule for tuning
            self.pub_pid_x.publish(self.pid_x.create_msg())
            self.pub_pid_vx.publish(self.pid_vx.create_msg())
            self.pub_pid_y.publish(self.pid_y.create_msg())
            self.pub_pid_vy.publish(self.pid_vy.create_msg())

    def odometry_cb(self, msg):
        '''
        Odometry (6DOF - position and orientation and velocities) callback.
        :param msg: Type Odometry
        '''

        if not self.start_flag:
            self.start_flag = True
        
        self.x_mv = msg.pose.pose.position.x
        self.y_mv = msg.pose.pose.position.y

        # transform linear velocity from body frame to inertial frame
        self.quaternion[0] = msg.pose.pose.orientation.x
        self.quaternion[1] = msg.pose.pose.orientation.y
        self.quaternion[2] = msg.pose.pose.orientation.z
        self.quaternion[3] = msg.pose.pose.orientation.w

        self.velocity_b[0] =  msg.twist.twist.linear.x
        self.velocity_b[1] =  msg.twist.twist.linear.y
        self.velocity_b[2] =  msg.twist.twist.linear.z

        Rib = transformations.quaternion_matrix(self.quaternion)
        Rv_b = transformations.translation_matrix(self.velocity_b)
        Rv_i = numpy.dot(Rib, Rv_b)
        self.velocity_i = transformations.translation_from_matrix(Rv_i)

        self.vx_mv = self.velocity_i[0]
        self.vy_mv = self.velocity_i[1]

        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w

        # conversion quaternion to euler (yaw - pitch - roll)
        #self.roll_mv = math.atan2(2 * (qw * qx + qy * qz), qw * qw - qx * qx - qy * qy + qz * qz)
        #self.pitch_mv = math.asin(2 * (qw * qy - qx * qz))
        self.yaw_mv = math.atan2(2 * (qw * qz + qx * qy), qw * qw + qx * qx - qy * qy - qz * qz)


    def vel_ref_cb(self, msg):
        '''
        Referent velocity callback. Use received velocity values when during initial tuning
        velocity controller (i.e. when position controller still not implemented).
        :param msg: Type Vector3
        '''
        self.vx_sp = msg.x
        self.vy_sp = msg.y

    def pos_ref_cb(self, msg):
        '''
        Referent position callback. Received value (z-component) is used as a referent height.
        :param msg: Type Vector3
        '''
        self.x_sp = msg.x
        self.y_sp = msg.y

    def yaw_ref_cb(self, msg):
        '''
        Referent yaw callback. Received value is used as a referent yaw (heading).
        :param msg: Type Float32
        '''
        self.yaw_sp = msg.data

    def cfg_callback(self, config, level):
        """
        Callback for dynamically reconfigurable parameters (P,I,D gains for height and velocity controller)
        """

        if not self.config_start:
            # callback is called for the first time. Use this to set the new params to the config server
            config.x_kp = self.pid_x.get_kp()
            config.x_ki = self.pid_x.get_ki()
            config.x_kd = self.pid_x.get_kd()

            config.vx_kp = self.pid_vx.get_kp()
            config.vx_ki = self.pid_vx.get_ki()
            config.vx_kd = self.pid_vx.get_kd()

            config.y_kp = self.pid_y.get_kp()
            config.y_ki = self.pid_y.get_ki()
            config.y_kd = self.pid_y.get_kd()

            config.vy_kp = self.pid_vy.get_kp()
            config.vy_ki = self.pid_vy.get_ki()
            config.vy_kd = self.pid_vy.get_kd()

            self.config_start = True
        else:
            # The following code just sets up the P,I,D gains for all controllers
            self.pid_x.set_kp(config.x_kp)
            self.pid_x.set_ki(config.x_ki)
            self.pid_x.set_kd(config.x_kd)

            self.pid_vx.set_kp(config.vx_kp)
            self.pid_vx.set_ki(config.vx_ki)
            self.pid_vx.set_kd(config.vx_kd)

            self.pid_y.set_kp(config.y_kp)
            self.pid_y.set_ki(config.y_ki)
            self.pid_y.set_kd(config.y_kd)

            self.pid_vy.set_kp(config.vy_kp)
            self.pid_vy.set_ki(config.vy_ki)
            self.pid_vy.set_kd(config.vy_kd)


        # this callback should return config data back to server
        return config

if __name__ == '__main__':

    rospy.init_node('mav_xy_controller')
    xy_ctl = HorizontalControl()
    xy_ctl.run()
