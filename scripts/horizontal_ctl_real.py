#!/usr/bin/env python

__author__ = 'thaus'

import rospy
from pid import PID
from geometry_msgs.msg import Vector3, PoseWithCovarianceStamped, PoseStamped, TwistStamped, Twist
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint
from std_msgs.msg import Float32, Empty
from dynamic_reconfigure.server import  Server
from urs_solution.cfg import MavXYCtlParamsConfig
from urs_solution.msg import PIDController
from tf import transformations
import numpy
import math
from sensor_msgs.msg import Joy

class HorizontalControl:
    '''
    Class implements ROS node for cascade (z, vz) PID control for MAV height.
    Subscribes to:
        pose       - used to extract z-position of the vehicle
        velocity   - used to extract vz of the vehicle
        pos_ref    - used to set the reference for z-position
        vel_ref    - used to set the reference for vz-position (useful for testing velocity controller)

    Publishes:
        euler_ref  - publishes referent value for euler angles (roll, pitch, yaw)
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

        # PID controllers for z-axis and yaw
        self.z_sp = 1.5
        self.z_mv = 0
        self.pid_z = PID()

        self.yaw_sp = 0
        self.pid_yaw = PID()

        x = 0
        #########################################################
        #########################################################
        # Add parameters for x controller
        self.pid_x.set_kp(0.8) # 0.2
        self.pid_x.set_ki(0.0) # 0.005
        self.pid_x.set_kd(0.0) # 0.3

        # Add parameters for vx controller
        self.pid_vx.set_kp(0.15) # 1
        self.pid_vx.set_ki(0) # 0
        self.pid_vx.set_kd(0) # 0

        # Add parameters for y controller
        self.pid_y.set_kp(0.8) # 0.2
        self.pid_y.set_ki(0.0) # 0.005
        self.pid_y.set_kd(0.0) # 0.3

        # Add parameters for vy controller
        self.pid_vy.set_kp(0.15) # 1
        self.pid_vy.set_ki(0) # 0
        self.pid_vy.set_kd(0) # 0

        #########################################################
        #########################################################

        # Add parameters for z controller
        self.pid_z.set_kp(0.4)
        self.pid_z.set_ki(0.05)
        self.pid_z.set_kd(0.0)

        # Add parameters for yaw controller
        self.pid_yaw.set_kp(1)
        self.pid_yaw.set_ki(0)
        self.pid_yaw.set_kd(0)

        self.pid_x.set_lim_up(5)        # max vx speed
        self.pid_x.set_lim_low(-5)      # min vx speed

        self.pid_vx.set_lim_up(20.0/180 * math.pi)        # max pitch angle
        self.pid_vx.set_lim_low(-20.0/180 * math.pi)      # min pitch angle

        self.pid_y.set_lim_up(5)        # max vy speed
        self.pid_y.set_lim_low(-5)      # min vy speed

        self.pid_vy.set_lim_up(20.0/180 * math.pi)        # max roll angle
        self.pid_vy.set_lim_low(-20.0/180 * math.pi)      # min roll angle

        self.pid_z.set_lim_up(1)
        self.pid_z.set_lim_low(-1)

        self.pid_yaw.set_lim_up(1)
        self.pid_yaw.set_lim_low(-1)



        rospy.Subscriber('optitrack/pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('optitrack/velocity', TwistStamped, self.vel_cb)
        rospy.Subscriber('vel_ref', Vector3, self.vel_ref_cb)
        rospy.Subscriber('pos_ref', Vector3, self.pos_ref_cb)
        rospy.Subscriber("joy", Joy, self.JoyCallback)
	rospy.Subscriber('trajectory_point_ref', MultiDOFJointTrajectoryPoint, self.trajectory_point_ref_cb)
        self.pub_pid_x = rospy.Publisher('pid_x', PIDController, queue_size=1)
        self.pub_pid_vx = rospy.Publisher('pid_vx', PIDController, queue_size=1)
        self.pub_pid_y = rospy.Publisher('pid_y', PIDController, queue_size=1)
        self.pub_pid_vy = rospy.Publisher('pid_vy', PIDController, queue_size=1)
        self.pub_pid_z = rospy.Publisher('pid_z', PIDController, queue_size=1)
        self.pub_pid_yaw = rospy.Publisher('pid_yaw', PIDController, queue_size=1)
        self.euler_ref_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.takeoffPub = rospy.Publisher("ardrone/takeoff", Empty, queue_size=1)
        self.landPub = rospy.Publisher("ardrone/land", Empty, queue_size=1)
        self.resetPub = rospy.Publisher("ardrone/reset", Empty, queue_size=1)

        self.cfg_server = Server(MavXYCtlParamsConfig, self.cfg_callback)
        self.ros_rate = rospy.Rate(20)
        self.t_start = rospy.Time.now()

        # Joy stuff
        self.joyMasterFlag = False

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

            # If joystick control is disabled automatic control is on
            if self.joyMasterFlag == False:
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

                vx_ref = self.pid_x.compute(self.x_sp, self.x_mv)+self.velocity_ff.x
                vy_ref = self.pid_y.compute(self.y_sp, self.y_mv)+self.velocity_ff.y
                pitch_sp = self.pid_vx.compute(vx_ref, self.vx_mv)+self.acceleration_ff.x*0.0
                roll_sp = self.pid_vy.compute(vy_ref, self.vy_mv)+self.acceleration_ff.y*0.0

                ########################################################
                ########################################################

                # Z and YAW controllers
                thrust_sp = self.pid_z.compute(self.z_sp, self.z_mv)
                yaw = self.pid_yaw.compute(self.yaw_sp, self.yaw_mv)

                #euler_sv = Vector3(roll_sp, pitch_sp, 0)
                euler_sv = Twist()
                #roll_sp = 0
                #pitch_sp = 0
                # Scaling roll and pitch values to 1
                euler_sv.linear.y = roll_sp/0.35
                euler_sv.linear.x = pitch_sp/0.35
                euler_sv.linear.z = thrust_sp
                euler_sv.angular.z = yaw
                self.euler_ref_pub.publish(euler_sv)

                # Publish PID data - could be usefule for tuning
                self.pub_pid_x.publish(self.pid_x.create_msg())
                self.pub_pid_vx.publish(self.pid_vx.create_msg())
                self.pub_pid_y.publish(self.pid_y.create_msg())
                self.pub_pid_vy.publish(self.pid_vy.create_msg())
                self.pub_pid_z.publish(self.pid_z.create_msg())
                self.pub_pid_yaw.publish(self.pid_yaw.create_msg())

            else:
                self.pid_x.reset()
                self.pid_y.reset()
                self.pid_vx.reset()
                self.pid_vy.reset()
                self.pid_z.reset()
                self.pid_yaw.reset()
                
    def trajectory_point_ref_cb(self, msg):
        '''
        Callback for one trajectory point with speed and acceleration
        '''

        # translation is vector3, we need point
        self.x_sp = msg.transforms[0].translation.x
        self.y_sp = msg.transforms[0].translation.y
        self.z_sp = msg.transforms[0].translation.z

        # orientation
        self.orientation_sp = msg.transforms[0].rotation

        # velocity and acceleration
        self.velocity_ff = msg.velocities[0].linear
        self.acceleration_ff = msg.accelerations[0].linear
    
    def pose_cb(self, msg):
        '''
        Pose (6DOF - position and orientation) callback.
        :param msg: Type PoseWithCovarianceStamped
        '''
        self.x_mv = msg.pose.position.x
        self.y_mv = msg.pose.position.y
        self.z_mv = msg.pose.position.z

        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w

        # conversion quaternion to euler (yaw - pitch - roll)
        #self.roll_mv = math.atan2(2 * (qw * qx + qy * qz), qw * qw - qx * qx - qy * qy + qz * qz)
        #self.pitch_mv = math.asin(2 * (qw * qy - qx * qz))
        self.yaw_mv = math.atan2(2 * (qw * qz + qx * qy), qw * qw + qx * qx - qy * qy - qz * qz)
        self.yaw_mv = msg.pose.orientation.z
        
    def vel_cb(self, msg):
        '''
        Velocity callback (linear velocity - x,y,z)
        :param msg: Type Vector3Stamped
        '''
        if not self.start_flag:
            self.start_flag = True
        self.vx_mv = msg.twist.linear.x
        self.vy_mv = msg.twist.linear.y

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
        self.z_sp = msg.z

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

    def JoyCallback(self, data):
        self.joyMasterData = data
        parrotCmdVel = Twist()

        if data.buttons[4] == 1:
            self.joyMasterFlag = True
        else:
            self.joyMasterFlag = False

        if self.joyMasterFlag == True:
            parrotCmdVel.linear.x = data.axes[3]
            parrotCmdVel.linear.y = data.axes[2]
            parrotCmdVel.linear.z = data.axes[1]
            parrotCmdVel.angular.z = data.axes[0]
            self.euler_ref_pub.publish(parrotCmdVel)

        if data.buttons[7] == 1:
            self.takeoffPub.publish(Empty())
            self.x_sp = self.x_mv
            self.y_sp = self.y_mv

        if data.buttons[6] == 1:
            self.landPub.publish(Empty())

        if data.buttons[8] == 1:
            self.resetPub.publish(Empty())


if __name__ == '__main__':

    rospy.init_node('mav_xy_controller')
    xy_ctl = HorizontalControl()
    xy_ctl.run()
