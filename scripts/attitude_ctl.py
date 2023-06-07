#!/usr/bin/env python

__author__ = 'thaus'

import rospy
from pid import PID
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from mav_msgs.msg import Actuators
from dynamic_reconfigure.server import Server
from med_uav_control.msg import PIDController
from med_uav_control.cfg import MavAttitudeCtlParamsConfig
import math

class AttitudeControl:
    '''
    Class implements MAV attitude control (roll, pitch, yaw). Two PIDs in cascade are
    used for each degree of freedom.
    Subscribes to:
        odometry           - used to extract attitude and attitude rate of the vehicle
        mot_vel_ref        - used to receive reference motor velocity from the height controller
        euler_ref          - used to set the attitude reference (useful for testing controllers)
    Publishes:
        command/motors     - reference motor velocities sent to each motor controller
        pid_roll           - publishes PID-roll data - reference value, measured value, P, I, D and total component (useful for tuning params)
        pid_roll_rate      - publishes PID-roll_rate data - reference value, measured value, P, I, D and total component (useful for tuning params)
        pid_pitch          - publishes PID-pitch data - reference value, measured value, P, I, D and total component (useful for tuning params)
        pid_pitch_rate     - publishes PID-pitch_rate data - reference value, measured value, P, I, D and total component (useful for tuning params)
        pid_yaw            - publishes PID-yaw data - reference value, measured value, P, I, D and total component (useful for tuning params)
        pid_yaw_rate       - publishes PID-yaw_rate data - reference value, measured value, P, I, D and total component (useful for tuning params)

    Dynamic reconfigure is used to set controllers param online.
    '''

    def __init__(self):
        '''
        Initialization of the class.
        '''

        self.start_flag = False             # flag indicates if the first measurement is received
        self.config_start = False           # flag indicates if the config callback is called for the first time
        self.euler_mv = Vector3()           # measured euler angles
        self.euler_sp = Vector3(0, 0, 0)    # euler angles reference values

        self.w_sp = 0                       # reference value for motor velocity - it should be the output of height controller

        self.euler_rate_mv = Vector3()      # measured angular velocities

        self.pid_roll = PID()                           # roll controller
        self.pid_roll_rate  = PID()                     # roll rate (wx) controller

        self.pid_pitch = PID()                          # pitch controller
        self.pid_pitch_rate = PID()                     # pitch rate (wy) controller

        self.pid_yaw = PID()                            # yaw controller
        self.pid_yaw_rate = PID()                       # yaw rate (wz) controller

        ##################################################################
        ##################################################################
        # Add your PID params here

        # kut
        self.pid_roll.set_kp(10)
        self.pid_roll.set_ki(0.25)
        self.pid_roll.set_kd(0.25)
        # kutna brzina
        self.pid_roll_rate.set_kp(50)
        self.pid_roll_rate.set_ki(50)
        self.pid_roll_rate.set_kd(0)
        # kut
        self.pid_pitch.set_kp(10)
        self.pid_pitch.set_ki(0.25)
        self.pid_pitch.set_kd(0.25)
        # kutna brzina
        self.pid_pitch_rate.set_kp(50)
        self.pid_pitch_rate.set_ki(50)
        self.pid_pitch_rate.set_kd(0)
        #kut
        self.pid_yaw.set_kp(2.5)
        self.pid_yaw.set_ki(1)
        self.pid_yaw.set_kd(0.1)
        # kutna brzina
        self.pid_yaw_rate.set_kp(30)
        self.pid_yaw_rate.set_ki(0)
        self.pid_yaw_rate.set_kd(0)

        ##################################################################
        ##################################################################

        self.ros_rate = rospy.Rate(100)                 # attitude control at 100 Hz

        rospy.Subscriber('odometry', Odometry, self.odometry_cb)
        rospy.Subscriber('mot_vel_ref', Float32, self.mot_vel_ref_cb)
        rospy.Subscriber('euler_ref', Vector3, self.euler_ref_cb)
        self.pub_mot = rospy.Publisher('/gazebo/command/motor_speed', Actuators, queue_size=1)
        self.pub_pid_roll = rospy.Publisher('pid_roll', PIDController, queue_size=1)
        self.pub_pid_roll_rate = rospy.Publisher('pid_roll_rate', PIDController, queue_size=1)
        self.pub_pid_pitch = rospy.Publisher('pid_pitch', PIDController, queue_size=1)
        self.pub_pid_pitch_rate = rospy.Publisher('pid_pitch_rate', PIDController, queue_size=1)
        self.pub_pid_yaw = rospy.Publisher('pid_yaw', PIDController, queue_size=1)
        self.pub_pid_yaw_rate = rospy.Publisher('pid_yaw_rate', PIDController, queue_size=1)
        self.cfg_server = Server(MavAttitudeCtlParamsConfig, self.cfg_callback)

    def run(self):
        '''
        Runs ROS node - computes PID algorithms for cascade attitude control.
        '''

        while not self.start_flag:
            print("Waiting for the first measurement.")
            rospy.sleep(0.5)
        print("Starting attitude control.")

        while not rospy.is_shutdown():
            self.ros_rate.sleep()

            ####################################################################
            ####################################################################
            # Add your code for cascade control for roll, pitch, yaw.
            # reference attitude values are stored in self.euler_sp
            # (self.euler_sp.x - roll, self.euler_sp.y - pitch, self.euler_sp.z - yaw)
            # Measured attitude values are stored in self.euler_mv (x,y,z - roll, pitch, yaw)
            # Measured attitude rate values are store in self.euler_rate_mv (self.euler_rate_mv.x, y, z)
            # Your result should be reference velocity value for each motor
            # Store them in variables mot_sp1, mot_sp2, mot_sp3, mot_sp4

            roll_rate_sv = self.pid_roll.compute(self.euler_sp.x, self.euler_mv.x)
            # roll rate pid compute
            dw_roll = self.pid_roll_rate.compute(roll_rate_sv, self.euler_rate_mv.x)

            pitch_rate_sv = self.pid_pitch.compute(self.euler_sp.y, self.euler_mv.y)
            # pitch rate pid compute
            dw_pitch = self.pid_pitch_rate.compute(pitch_rate_sv, self.euler_rate_mv.y)

            # setting yaw reference to -pi,pi range
            yaw_ref = math.fmod(self.euler_sp.z, 2 * math.pi)
            if yaw_ref > math.pi:
                yaw_ref = yaw_ref - 2 * math.pi
            elif yaw_ref < -math.pi:
                yaw_ref = yaw_ref + 2 * math.pi

            # determine cw or ccw rotation
            dyaw = yaw_ref - self.euler_mv.z
            if dyaw > math.pi:
                yaw_ref = yaw_ref - 2 * math.pi
            elif dyaw < -math.pi:
                yaw_ref = yaw_ref + 2 * math.pi

            yaw_rate_sv = self.pid_yaw.compute(yaw_ref, self.euler_mv.z)
            # yaw rate pid compute
            dw_yaw = self.pid_yaw_rate.compute(yaw_rate_sv, self.euler_rate_mv.z)

            # Moments and forces distribution
            mot_sp1 = self.w_sp - dw_roll - dw_pitch - dw_yaw # reference value for motor velocity w.sp
            mot_sp2 = self.w_sp + dw_roll - dw_pitch + dw_yaw
            mot_sp3 = self.w_sp + dw_roll + dw_pitch - dw_yaw
            mot_sp4 = self.w_sp - dw_roll + dw_pitch + dw_yaw

            ####################################################################
            ####################################################################

            # Publish motor velocities
            mot_speed_msg = Actuators()
            mot_speed_msg.angular_velocities = [mot_sp1,mot_sp2,mot_sp3,mot_sp4]
            self.pub_mot.publish(mot_speed_msg)

            # Publish PID data - could be usefule for tuning
            self.pub_pid_roll.publish(self.pid_roll.create_msg())
            self.pub_pid_roll_rate.publish(self.pid_roll_rate.create_msg())
            self.pub_pid_pitch.publish(self.pid_pitch.create_msg())
            self.pub_pid_pitch_rate.publish(self.pid_pitch_rate.create_msg())
            self.pub_pid_yaw.publish(self.pid_yaw.create_msg())
            self.pub_pid_yaw_rate.publish(self.pid_yaw_rate.create_msg())

    def mot_vel_ref_cb(self, msg):
        '''
        reference motor velocity callback. (This should be published by height controller).
        :param msg: Type Float32
        '''
        self.w_sp = msg.data # reference value for motor velocity

    def odometry_cb(self, msg):
        '''
        Odometry callback. Used to extract roll, pitch, yaw and their rates.
        We used the following order of rotation - 1)yaw, 2) pitch, 3) roll
        :param msg: Type nav_msgs/Odometry
        '''
        if not self.start_flag:
            self.start_flag = True

        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w

        # conversion quaternion to euler (yaw - pitch - roll)
        self.euler_mv.x = math.atan2(2 * (qw * qx + qy * qz), qw * qw - qx * qx - qy * qy + qz * qz)
        self.euler_mv.y = math.asin(2 * (qw * qy - qx * qz))
        self.euler_mv.z = math.atan2(2 * (qw * qz + qx * qy), qw * qw + qx * qx - qy * qy - qz * qz)

        # gyro measurements (p,q,r)
        p = msg.twist.twist.angular.x
        q = msg.twist.twist.angular.y
        r = msg.twist.twist.angular.z

        sx = math.sin(self.euler_mv.x)   # sin(roll)
        cx = math.cos(self.euler_mv.x)   # cos(roll)
        cy = math.cos(self.euler_mv.y)   # cos(pitch)
        ty = math.tan(self.euler_mv.y)   # cos(pitch)

        # conversion gyro measurements to roll_rate, pitch_rate, yaw_rate
        self.euler_rate_mv.x = p + sx * ty * q + cx * ty * r
        self.euler_rate_mv.y = cx * q - sx * r
        self.euler_rate_mv.z = sx / cy * q + cx / cy * r

    def pose_cb(self, msg):
        '''
        Pose (6DOF - position and orientation) callback.
        :param msg: Type PoseWithCovarianceStamped
        '''
        return

    def euler_ref_cb(self, msg):
        '''
        Euler ref values callback.
        :param msg: Type Vector3 (x-roll, y-pitch, z-yaw)
        '''
        self.euler_sp = msg

    def cfg_callback(self, config, level):
        """ Callback for dynamically reconfigurable parameters (P,I,D gains for each controller)
        """

        if not self.config_start:
            # callback is called for the first time. Use this to set the new params to the config server
            config.roll_kp = self.pid_roll.get_kp()
            config.roll_ki = self.pid_roll.get_ki()
            config.roll_kd = self.pid_roll.get_kd()

            config.roll_r_kp = self.pid_roll_rate.get_kp()
            config.roll_r_ki = self.pid_roll_rate.get_ki()
            config.roll_r_kd = self.pid_roll_rate.get_kd()

            config.pitch_kp = self.pid_pitch.get_kp()
            config.pitch_ki = self.pid_pitch.get_ki()
            config.pitch_kd = self.pid_pitch.get_kd()

            config.pitch_r_kp = self.pid_pitch_rate.get_kp()
            config.pitch_r_ki = self.pid_pitch_rate.get_ki()
            config.pitch_r_kd = self.pid_pitch_rate.get_kd()

            config.yaw_kp = self.pid_yaw.get_kp()
            config.yaw_ki = self.pid_yaw.get_ki()
            config.yaw_kd = self.pid_yaw.get_kd()

            config.yaw_r_kp = self.pid_yaw_rate.get_kp()
            config.yaw_r_ki = self.pid_yaw_rate.get_ki()
            config.yaw_r_kd = self.pid_yaw_rate.get_kd()

            self.config_start = True
        else:
            # The following code just sets up the P,I,D gains for all controllers
            self.pid_roll.set_kp(config.roll_kp)
            self.pid_roll.set_ki(config.roll_ki)
            self.pid_roll.set_kd(config.roll_kd)

            self.pid_roll_rate.set_kp(config.roll_r_kp)
            self.pid_roll_rate.set_ki(config.roll_r_ki)
            self.pid_roll_rate.set_kd(config.roll_r_kd)

            self.pid_pitch.set_kp(config.pitch_kp)
            self.pid_pitch.set_ki(config.pitch_ki)
            self.pid_pitch.set_kd(config.pitch_kd)

            self.pid_pitch_rate.set_kp(config.pitch_r_kp)
            self.pid_pitch_rate.set_ki(config.pitch_r_ki)
            self.pid_pitch_rate.set_kd(config.pitch_r_kd)

            self.pid_yaw.set_kp(config.yaw_kp)
            self.pid_yaw.set_kp(config.yaw_kp)
            self.pid_yaw.set_ki(config.yaw_ki)
            self.pid_yaw.set_kd(config.yaw_kd)

            self.pid_yaw_rate.set_kp(config.yaw_r_kp)
            self.pid_yaw_rate.set_ki(config.yaw_r_ki)
            self.pid_yaw_rate.set_kd(config.yaw_r_kd)

        # this callback should return config data back to server
        return config

if __name__ == '__main__':

    rospy.init_node('mav_attitude_ctl')
    attitude_ctl = AttitudeControl()
    attitude_ctl.run()