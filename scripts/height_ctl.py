#!/usr/bin/env python

__author__ = 'thaus'

import rospy
from pid import PID
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from tf import transformations
import numpy
from dynamic_reconfigure.server import  Server
from med_uav_control.cfg import MavZCtlParamsConfig
from med_uav_control.msg import PIDController
from mav_msgs.msg import Actuators

class HeightControl:
    '''
    Class implements ROS node for cascade (z, vz) PID control for MAV height.
    Subscribes to:
        odometry    - used to extract z-position and velocitr of the vehicle
        pos_ref    - used to set the reference for z-position
        vel_ref    - used to set the reference for vz-position (useful for testing velocity controller)

    Publishes:
        mot_vel_ref  - referent value for thrust in terms of motor velocity (rad/s)
        pid_z        - publishes PID-z data - referent value, measured value, P, I, D and total component (useful for tuning params)
        pid_vz        - publishes PID-vz data - referent value, measured value, P, I, D and total component (useful for tuning params)

    Dynamic reconfigure is used to set controller params online.
    '''

    def __init__(self):
        '''
        Initialization of the class.
        '''

        self.start_flag = False         # indicates if we received the first measurement
        self.config_start = False       # flag indicates if the config callback is called for the first time

        self.z_sp = 0                   # z-position set point
        self.z_mv = 0                   # z-position measured value
        self.pid_z = PID()              # pid instance for z control

        self.vz_sp = 0                  # vz velocity set_point
        self.vz_mv = 0                   # vz velocity measured value
        self.pid_vz = PID()             # pid instance for z-velocity control

        #########################################################
        #########################################################
        
        # Add parameters for vz controller
        self.pid_vz.set_kp(75)
        self.pid_vz.set_ki(10)
        self.pid_vz.set_kd(0.41472)

        # Add parameters for z controller
        self.pid_z.set_kp(0.5)
        self.pid_z.set_ki(0.125)
        self.pid_z.set_kd(0)

        #########################################################
        #########################################################

        self.pid_vz.set_lim_up(1400)    # max velocity of a motor
        self.pid_vz.set_lim_low(0)      # min velocity of a motor

        self.pid_z.set_lim_up(5)        # max vertical ascent speed
        self.pid_z.set_lim_low(-5)      # max vertical descent speed

        self.mot_speed = 0      # referent motors velocity, computed by PID cascade

        self.quaternion = numpy.empty((4, ), dtype=numpy.float64)
        self.velocity_b = numpy.empty((3, ), dtype=numpy.float64)
        self.velocity_i = numpy.empty((3, ), dtype=numpy.float64)

        self.attitude_ctl = rospy.get_param('~attitude_control', 1)   # flag indicates if attitude control is turned on

        rospy.Subscriber('odometry', Odometry, self.odometry_cb)
        rospy.Subscriber('vel_ref', Vector3, self.vel_ref_cb)
        rospy.Subscriber('pos_ref', Vector3, self.pos_ref_cb)
        self.pub_pid_z = rospy.Publisher('pid_z', PIDController, queue_size=1)
        self.pub_pid_vz = rospy.Publisher('pid_vz', PIDController, queue_size=1)
        self.mot_ref_pub = rospy.Publisher('mot_vel_ref', Float32, queue_size=1)
        self.pub_mot = rospy.Publisher('/gazebo/command/motor_speed', Actuators, queue_size=1)
        self.cfg_server = Server(MavZCtlParamsConfig, self.cfg_callback)
        self.ros_rate = rospy.Rate(20)
        self.t_start = rospy.Time.now()

    def run(self):
        '''
        Runs ROS node - computes PID algorithms for z and vz control.
        '''

        while not self.start_flag:
            print('Waiting for velocity measurements.')
            rospy.sleep(0.5)
        print("Starting height control.")

        while not rospy.is_shutdown():
            self.ros_rate.sleep()

            ########################################################
            ########################################################
            # Implement cascade PID control here.
            # Reference for z is stored in self.z_sp.
            # Measured z-position is stored in self.z_mv.
            # If you want to test only vz - controller, the corresponding reference is stored in self.vz_sp.
            # Measured vz-velocity is stored in self.vz_mv
            # Resultant referent value for motor velocity should be stored in variable mot_speed.

            self.mot_speed_hover = 402.5
            vz_ref = self.pid_z.compute(self.z_sp, self.z_mv)

            self.mot_speed = self.mot_speed_hover + \
                        self.pid_vz.compute(vz_ref, self.vz_mv)

            ########################################################
            ########################################################


            if self.attitude_ctl == 0:
                # Publish motor velocities
                mot_speed_msg = Actuators()
                mot_speed_msg.angular_velocities = [self.mot_speed,self.mot_speed,self.mot_speed, self.mot_speed]
                self.pub_mot.publish(mot_speed_msg)
            else:
                # publish reference motor velocity to attitude controller
                mot_speed_msg = Float32(self.mot_speed)
                self.mot_ref_pub.publish(mot_speed_msg)


            # Publish PID data - could be useful for tuning
            self.pub_pid_z.publish(self.pid_z.create_msg())
            self.pub_pid_vz.publish(self.pid_vz.create_msg())

    def pose_cb(self, msg):
        '''
        Pose (6DOF - position and orientation) callback.
        :param msg: Type PoseWithCovarianceStamped
        '''
        self.z_mv = msg.pose.position.z

    def odometry_cb(self, msg):
        '''
        Odometry (6DOF - position and orientation and velocities) callback.
        :param msg: Type Odometry
        '''

        if not self.start_flag:
            self.start_flag = True
        
        self.z_mv = msg.pose.pose.position.z

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

        self.vz_mv = self.velocity_i[2]

    def vel_cb(self, msg):
        '''
        Velocity callback (linear velocity - x,y,z)
        :param msg: Type Vector3Stamped
        '''
        if not self.start_flag:
            self.start_flag = True
        self.vz_mv = msg.twist.linear.z

    def vel_ref_cb(self, msg):
        '''
        Referent velocity callback. Use received velocity values when during initial tuning
        velocity controller (i.e. when position controller still not implemented).
        :param msg: Type Vector3
        '''
        self.vz_sp = msg.z

    def pos_ref_cb(self, msg):
        '''
        Referent position callback. Received value (z-component) is used as a referent height.
        :param msg: Type Vector3
        '''
        self.z_sp = msg.z

    def cfg_callback(self, config, level):
        """
        Callback for dynamically reconfigurable parameters (P,I,D gains for height and velocity controller)
        """

        if not self.config_start:
            # callback is called for the first time. Use this to set the new params to the config server
            config.z_kp = self.pid_z.get_kp()
            config.z_ki = self.pid_z.get_ki()
            config.z_kd = self.pid_z.get_kd()

            config.vz_kp = self.pid_vz.get_kp()
            config.vz_ki = self.pid_vz.get_ki()
            config.vz_kd = self.pid_vz.get_kd()

            self.config_start = True
        else:
            # The following code just sets up the P,I,D gains for all controllers
            self.pid_z.set_kp(config.z_kp)
            self.pid_z.set_ki(config.z_ki)
            self.pid_z.set_kd(config.z_kd)

            self.pid_vz.set_kp(config.vz_kp)
            self.pid_vz.set_ki(config.vz_ki)
            self.pid_vz.set_kd(config.vz_kd)

        # this callback should return config data back to server
        return config

if __name__ == '__main__':

    rospy.init_node('mav_z_controller')
    height_ctl = HeightControl()
    height_ctl.run()

