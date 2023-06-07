#!/usr/bin/env python

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
from sim_controller import UAV

'''

var yaw = atan2(2.0*(q.y*q.z + q.w*q.x), q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z);
var pitch = asin(-2.0*(q.x*q.z - q.w*q.y));
var roll = atan2(2.0*(q.x*q.y + q.w*q.z), q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z);

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
        self.odometry = None
        self.pos_ref = None
        self.ros_rate = rospy.Rate(100) # geometric control at 100 Hz
        rospy.Subscriber('odometry', Odometry, self.odometry_cb)
        rospy.Subscriber('pos_ref', Vector3, self.odometry_cb)
        self.mot_ref_pub = rospy.Publisher('mot_vel_ref', Float32, queue_size=1)

        # initialize UAV class

    def odometry_cb(self, odom):
        # pohraniti u odometriju
        self.odometry = odom
        print("ODOMETRIJA:", odom)

    def pos_ref_cb(self, pos):
        self.pos_ref = pos

    def run(self):
        # inicijalizirati vektor stanja X iz odometrije
        # pozivati dydt
        # pomocu f, M dobivenih od dydt upravljati letjelicom (pretvoriti u brzine vrtnje)
        while not rospy.is_shutdown():
            self.ros_rate.sleep()
        # pogledati kakva je rotacijska matrica kad se letjelica mice!
        
        # mot_speed_msg = Actuators()
        # mot_speed_msg.angular_velocities = [self.mot_speed,self.mot_speed,self.mot_speed, self.mot_speed]
        # self.pub_mot.publish(mot_speed_msg)

if __name__ == '__main__':

    rospy.init_node('geometric_controller')
    geom_ctl = GeometricController()
    geom_ctl.run()