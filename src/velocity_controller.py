#! /usr/bin/env python

import math
from numpy.core.arrayprint import dtype_is_implied
from numpy.core.function_base import geomspace
from numpy.core.overrides import verify_matching_signatures
from numpy.lib.function_base import select
import rospy
from geometry_msgs.msg import Twist, Point, Quaternion
import tf
from math import radians, sqrt, pow, pi

import matplotlib.pyplot as plt
from math import pi

import numpy as np


import sys


class RobotController():
    def __init__(self, *args, **kwargs):
        # Give the node a name
        rospy.init_node('velocity_controller', anonymous=False)

        # Set rospy to execute a shutdown function when terminating the script
        rospy.on_shutdown(self.shutdown)


        self.new_velocity_sub = rospy.Publisher('/change', Twist, queue_size=1)


        # How fast will we check the odometry values?
        rate = 10

        # Set the equivalent ROS rate variable
        self.r = rospy.Rate(rate)

        # Publisher to control the robot's speed
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)

        # The base frame is base_footprint for the TurtleBot but base_link for Pi Robot
        self.base_frame = '/base_link'

        # The odom frame is usually just /odom
        self.odom_frame = '/odom'

        # Initialize the tf listener
        self.tf_listener = tf.TransformListener()

        # Give tf some time to fill its buffer
        rospy.sleep(2)

        self.odom_frame = '/odom'


        # Find out if the robot uses /base_link or /base_footprint
        try:
            self.tf_listener.waitForTransform(
                self.odom_frame, '/base_footprint', rospy.Time(), rospy.Duration(1.0))
            self.base_frame = '/base_footprint'
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            try:
                self.tf_listener.waitForTransform(
                    self.odom_frame, '/base_link', rospy.Time(), rospy.Duration(1.0))
                self.base_frame = '/base_link'
            except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                rospy.loginfo(
                    "Cannot find transform between /odom and /base_link or /base_footprint")
                rospy.signal_shutdown("tf Exception")

    def quat_to_angle(self, quat):
        return tf.transformations.euler_from_quaternion((quat.x, quat.y, quat.z, quat.w))[2]

    def normalize_angle(self, angle):
        res = angle
        while res > pi:
            res -= 2.0 * pi
        while res < -pi:
            res += 2.0 * pi
        return res

   

    def move(self):

        (position, self.new_theta) = self.get_odom()
        x, y = position.x, position.y
        nearest_point = self.nearest(x, y)
        mapp_x, mapp_y = self.mapp
        sum_i = 0
        n_x, n_y = round(mapp_x[nearest_point], 6), round(
            mapp_y[nearest_point], 4)

        move_cmd = Twist()
        move_cmd.angular.z = 0
        # Set the movement command to forward motion
        move_cmd.linear.x = self.v

        print(f'first point : {(n_x,n_y)}')

        while True:
            self.cmd_vel.publish(move_cmd)
            self.new_velocity_sub.publish(move_cmd)

            delta_x, delta_y = n_x - x, n_y - y

            v_err = np.sqrt(np.power(delta_x, 2) + np.power(delta_y, 2))
            
            self.errors.append(v_err)
            
            self.v = min(self.k_p * v_err + self.k_i * sum_i , 0.6)

            sum_i += v_err * self.dt

            theta_star = math.atan2(n_y - y, n_x - x)

            delta_theta = theta_star - self.theta

            delta_theta = self.normalize_angle(delta_theta)

            delta_theta = self.k_teta * delta_theta

            move_cmd.angular.z = delta_theta
            move_cmd.linear.x = self.v

            if v_err <= self.d_star:
                # self.d_star = v_err
                nearest_point = (nearest_point + 1) % len(mapp_x)
                # print(nearest_point)
                n_x, n_y = round(mapp_x[nearest_point], 6), round(
                    mapp_y[nearest_point], 6)
                print(f'next point : {(n_x,n_y,nearest_point)}')

            (position, self.theta) = self.get_odom()
            x, y = position.x, position.y

            self.r.sleep()

    def get_odom(self):
        # Get the current transform between the odom and base frames

        try:
            (trans, rot) = self.tf_listener.lookupTransform(
                self.odom_frame, self.base_frame, rospy.Time(0))
            if self.capture_error:
                point = Point(*trans)
                self.errors.append(self.path_error(point.x, point.y))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return

        return (Point(*trans), self.quat_to_angle(Quaternion(*rot)))

   
    def shutdown(self):
        # Always stop the robot when shutting down the node
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)

    

if __name__ == '__main__':
  

    pass