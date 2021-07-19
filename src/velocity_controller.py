#! /usr/bin/python3

import math

import rospy
from geometry_msgs.msg import Twist, Point, Quaternion
import tf
from math import radians, sqrt, pow, pi

import matplotlib.pyplot as plt
from math import pi

import numpy as np


from nav_msgs.msg import OccupancyGrid


import sys


ROTATION , MOVE = range(2)

class RobotController():
    def __init__(self, *args, **kwargs):
        # Give the node a name
        rospy.init_node('velocity_controller', anonymous=False)

        # Set rospy to execute a shutdown function when terminating the script
        rospy.on_shutdown(self.shutdown)


        self.new_velocity_sub = rospy.Publisher('/change', Twist, queue_size=1)


        # How fast will we check the odometry values?
        rate = 20

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

        self.angular_speed = 0.3
        
        self.linear_speed = 0.4
        
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
                
        self.grid_sub = rospy.Subscriber('/window_data' , OccupancyGrid , self.callback_grid)
        
        self.angle_increment = 0.017501922324299812
        
        self.angle_max = 6.28318977355957
        
        self.number_of_sectors = int(self.angle_max/self.angle_increment) + 1
        
        self.smoothing_factors = [1,2,3,4,5,4,3,2,1]
        
        self.target_point = (0,0)
        
        self.state = MOVE
        
        self.smax = 50
        
        #TODO
        # tune the funcking threshold
        self.threshold = 150
        
        tw_msg = Twist()
        
        tw_msg.linear.x = self.linear_speed
        
        self.cmd_vel.publish(tw_msg)

    def quat_to_angle(self, quat):
        return tf.transformations.euler_from_quaternion((quat.x, quat.y, quat.z, quat.w))[2]

    def normalize_angle(self , angle):
        res = angle
        while res > pi:
            res -= 2.0 * pi
        while res < -pi:
            res += 2.0 * pi
        return res
    
    def scale_angle(self,angle):
        res = angle
        if res < 0:
            return res + (2 * pi)
        return res
    
    
    def calculate_magnitude(self, window_map):
        # we have two parameters A , B. 
        # assume << A + B = M>> to reduce the tunable parameters and calculations
        M = 1 # could be any real positive number
        
        N, _ = window_map.shape
        mid_map = (N - 1) / 2

        distances = np.zeros((N, N))

        d_max = mid_map * (2 ** 0.5)
        
        # << A - B * d_max = 0 >> and << A + B = M >>.
        # so << B = M / (1 + d_max) >> and << A = (M * d_max) / (1 + d_max) >>
        A = (M * d_max) / (1 + d_max)
        B = M / (1 + d_max)
        
        for i in range(N):
            for j in range(N):
                distances[i,j] = ((i - mid_map) ** 2 + (j - mid_map) ** 2) ** 0.5
        
        magnitude = (window_map ** 2) * (A - B * distances)
        return magnitude

   
    def callback_grid(self , msg):
        
        if self.state == ROTATION:
            return
        
        self.state = ROTATION
        
        tw_msg = Twist()
        
        rospy.sleep(0.2)
        
        self.cmd_vel.publish(tw_msg)
        
        
        dim = int(math.sqrt(len(msg.data)))
        # print(msg.data)
        mapp = np.array(list(msg.data)).reshape(dim,dim)
        
        mapp = self.calculate_magnitude(mapp)
        
        print(mapp.tolist())
        
        
        histogram = np.zeros((self.number_of_sectors,))
        
        
        # constructing a polar histogram
        mid = dim//2
        y = mid
        for i in range(dim):
            x = -mid
            for j in range(dim):
                sector = int(self.scale_angle(np.arctan2(y,x)) / self.angle_increment)
                histogram[sector] += mapp[i,j]
                x += 1
            y -= 1
                
        
        # smoothing 
        smooth_mid = len(self.smoothing_factors) // 2
        for i in range(len(histogram)):
            val = 0
            for j in range(-smooth_mid,smooth_mid):
                if i+j < 0 or i+j >= len(histogram):
                    continue
                val += histogram[i+j] * self.smoothing_factors[smooth_mid+j]
            val /= 2 * (smooth_mid+1) + 1
            histogram[i] = val
            
        
        # nearest valley
        valleys = histogram <= self.threshold
        # print(histogram.tolist())
        
        # print(valleys)
        
        pos , rotation = self.get_odom()
        
        ktarget = self.scale_angle(np.arctan2((pos.y - self.target_point[1]),(pos.x - self.target_point[0])))
        # ktarget
        
        ktarget = int(ktarget / self.angle_increment)
        
        i = ktarget - 1
        j = ktarget + 1
        
        goal_sector = -1
        
        while i >= 0 or j < len(histogram):
            
            if i >=0 and valleys[i]:
                ind = i
                kn = i
                valley_size = 1
                while ind >= 0 and valleys[ind]:
                    valley_size += 1
                    ind -= 1
                kf = -1
                if valley_size >= self.smax:
                    kf = kn - self.smax
                else:
                    kf = ind + 1
                    
                assert kf != -1
                goal_sector = (kf + kn) // 2
                break
                
            i -= 1
            
            if j < len(histogram) and valleys[j]:
                ind = j
                kn = j
                valley_size = 1
                while ind < len(histogram) and valleys[ind]:
                    valley_size += 1
                    ind += 1
                kf = -1
                if valley_size >= self.smax:
                    kf = kn + self.smax
                else:
                    kf = ind - 1
                
                assert kf != -1
                goal_sector = (kf + kn) // 2
                break
            
            j += 1
            
        goal_angle = self.normalize_angle(goal_sector * self.angle_increment)
        
        print("rotation : {} goal_angle : {}".format(rotation , goal_angle))
        
        tw_msg = Twist()
        
        last_angle = rotation
        
        turn_angle = 0
        
        tw_msg.angular.z = self.angular_speed * (rotation - goal_angle) / abs(goal_angle - rotation)
        
        while abs(turn_angle) < abs(goal_angle) and not rospy.is_shutdown():
            
            self.cmd_vel.publish(tw_msg)
            
            self.r.sleep()
            
            position , rotation = self.get_odom()
            
            delta_angle = self.normalize_angle(rotation - last_angle)
            
            turn_angle += delta_angle
            last_angle = rotation
            
        
        tw_msg = Twist()
        self.cmd_vel.publish(tw_msg)
        rospy.sleep(2)
        tw_msg = Twist()
        tw_msg.linear.x = self.linear_speed
        self.cmd_vel.publish(tw_msg)
        
        self.state = MOVE
        
        
    def get_odom(self):
        # Get the current transform between the odom and base frames

        try:
            (trans, rot) = self.tf_listener.lookupTransform(
                self.odom_frame, self.base_frame, rospy.Time(0))
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
    
    robot = RobotController()    

    rospy.spin()


