#!/usr/bin/env python
import sys, os
import rospy
import copy
import math
import time

from trajectory_msgs.msg import MultiDOFJointTrajectory
from geometry_msgs.msg import Vector3


class TrajectoryToPositionReference():

    def __init__(self):
        rospy.Subscriber('trajectory_reference', MultiDOFJointTrajectory, 
            self.trajectory_callback, queue_size=1)
        self.position_ref_pub = rospy.Publisher('pos_ref', Vector3, queue_size=1)

    def run(self):
        rospy.spin()

    def trajectory_callback(self, msg):
        trajectory = msg

        print("Trajectory received, length:", len(trajectory.points))
        rate = rospy.Rate(100)
        position_ref = Vector3()
        for i in range(len(trajectory.points)):
            rate.sleep()
            position_ref.x = trajectory.points[i].transforms[0].translation.x
            position_ref.y = trajectory.points[i].transforms[0].translation.y
            position_ref.z = trajectory.points[i].transforms[0].translation.z
            self.position_ref_pub.publish(position_ref)

if __name__=="__main__":
    rospy.init_node("TrajectoryToPositionReference")
    traj = TrajectoryToPositionReference()
    traj.run()

"""
Helix params:
r = 1.0
angleStep = 0.5
x0 = -1.0
y0 = 0.0
z0 = 1.0
zf = 2.5
deltaZ = 0.05
epsilon = 0.1
"""