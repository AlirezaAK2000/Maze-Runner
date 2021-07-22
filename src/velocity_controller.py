#!/usr/bin/python3

import math

from numpy.core.shape_base import block

import rospy
from geometry_msgs.msg import Twist, Point, Quaternion
from rospy.timer import sleep
import tf
from math import radians, sqrt, pow, pi

import matplotlib.pyplot as plt
from math import pi

import numpy as np

import matplotlib.pyplot as plt

from nav_msgs.msg import OccupancyGrid

from copy import deepcopy

import sys

import threading

import random

import datetime

ROTATION , MOVE = range(2)

class RobotController():
    def __init__(self, *args, **kwargs):
        # Give the node a name
        rospy.init_node('velocity_controller', anonymous=False)
        print(rospy.get_rostime())
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

        self.angular_speed = 0.3
        self.linear_speed = 0.25
        
        self.start_time = datetime.datetime.now()
        
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
                
        self.grid_sub = rospy.Subscriber('/window_data' , OccupancyGrid , self.callback_grid , queue_size=1)
        
        self.angle_increment = 0.01745330492655436 * 5
        
        self.angle_max = 6.28318977355957
        
        self.number_of_sectors = int(self.angle_max/self.angle_increment) + 1
        
        self.smooth_factor = 10
        
        self.smoothing_factors = list(range(1,self.smooth_factor)) + [self.smooth_factor] + list(reversed(range(1,self.smooth_factor))) 
        
        self.target_point = (-9,-6)
        
        
        
        self.state = MOVE
        self.end_rotation = rospy.get_rostime()
        
        self.smax = 12
        
        self.angular_tolerance = 0.2
        
        #TODO
        # tune the funcking threshold
        self.threshold = 8000
        
        tw_msg = Twist()
        
        tw_msg.linear.x = self.linear_speed
        
        self.cmd_vel.publish(tw_msg)
        
        # threading.Thread(target = self.goal_handler).start()
        
        
    def goal_handler(self):
        
        while True:
            pos , _  = self.get_odom()
            if math.sqrt((self.target_point[0] - pos.x)**2 + (self.target_point[1] - pos.y)** 2) < 2:
                print('target reached !!!')
                print(f'elapsed time : {datetime.datetime.now()-self.start_time}')
                self.grid_sub.shutdown()
                msg = Twist()
                self.cmd_vel.publish(msg)
            sleep(1)

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
        distances = np.zeros(window_map.shape)
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

   
    def callback_grid(self , msg:OccupancyGrid):
        if self.state == ROTATION or self.end_rotation >= msg.info.map_load_time:
            return
        
        self.state = ROTATION
        
        tw_msg = Twist()
        # how to avoid race condition in python
        self.cmd_vel.publish(tw_msg)
        
        rospy.sleep(1)
        
        resolution = msg.info.resolution
        width = msg.info.width
        height = msg.info.height
        
        
        mapp = np.array(list(msg.data)).reshape(width,height)
        
        mapp = self.calculate_magnitude(mapp)
        
        # print(mapp.tolist())
        
        
        histogram = np.zeros((self.number_of_sectors,))
        
        # constructing a polar histogram
        y = (-height // 2) * resolution
        for i in range(height):
            x = (-width // 2) * resolution
            for j in range(width):
                sector = int(self.scale_angle(np.arctan2(y,x)) / self.angle_increment)
                # print(sector)
                histogram[sector] += mapp[i,j]
                x += resolution
            y += resolution
                
                
        histcopy = deepcopy(histogram)
        
        # smoothing 
        smooth_mid = len(self.smoothing_factors) // 2
        for i in range(len(histogram)):
            val = 0
            for j in range(-smooth_mid,smooth_mid):
                val += histcopy[(i+j) % len(histogram) if i+j >= 0 else i+j] * self.smoothing_factors[smooth_mid+j]
            val /= 2 * (smooth_mid+1) + 1
            histogram[i] = val
            
        pos , rotation = self.get_odom()
        robot_heading_sector = int(self.scale_angle(rotation) / self.angle_increment)
        # ktarget =int(ktarget / self.angle_increment)
        ktarget =len(histogram) + robot_heading_sector
        
        i = ktarget - 1
        j = ktarget + 1
        
        goal_sector = None
        
        kn,kf = None,None
        
        
        histogram = np.array(histogram.tolist() + histogram.tolist() + histogram.tolist())
        # histogram = histogram[::-1]
        valleys = histogram <= self.threshold
        if not valleys[ktarget]:
            while i >= 0 or j < len(histogram):
                res = []
                if i >=0 and valleys[i]:
                    ind = i
                    kn = i
                    valley_size = 1
                    while ind >= 0 and valleys[ind]:
                        valley_size += 1
                        ind -= 1
                    if valley_size >= self.smax:
                        kf = kn - self.smax
                        print("wide valley")
                    else:
                        print("tight valley")
                        kf = ind + 1
                        
                    assert kf is not None
                    goal_sector = (kf + kn) // 2
                    print("kf : {} kn : {} ktarg : {} goal_sector : {}".format(
                        kf , kn , ktarget , goal_sector
                    ))
                    res.append((kf , kn , ktarget , goal_sector))
                i -= 1
                
                if j < len(histogram) and valleys[j]:
                    ind = j
                    kn = j
                    valley_size = 1
                    while ind < len(histogram) and valleys[ind]:
                        valley_size += 1
                        ind += 1
                    if valley_size >= self.smax:
                        print("wide valley")
                        kf = kn + self.smax
                    else:
                        print("tight valley")
                        kf = ind - 1
                    
                    assert kf is not None
                    goal_sector = (kf + kn) // 2
                    print("kf : {} kn : {} ktarg : {} goal_sector : {}".format(
                        kf , kn , ktarget , goal_sector
                    ))
                    res.append((kf , kn , ktarget , goal_sector))
                    
                j += 1
                assert len(res) <= 2
                if len(res) == 2:
                    (kf , kn , ktarget , goal_sector) = res[0] if abs(res[0][0] - res[0][1]) > abs(res[1][0] - res[1][1]) else res[1]
                    break
                elif len(res) == 1:
                    (kf , kn , ktarget , goal_sector) = res[0]
                    break
                # elif len(res) == 0:
                    
                    
        else:
            print("target in valley")
            res = []
            ind = i
            kn = i
            valley_size = 1
            while ind >= 0 and valleys[ind]:
                valley_size += 1
                ind -= 1
            if valley_size >= self.smax:
                kf = kn - self.smax
                print("wide valley")
            else:
                print("tight valley")
                kf = ind + 1
                
            assert kf is not None
            goal_sector = (kf + kn) // 2
            print("kf : {} kn : {} ktarg : {} goal_sector : {}".format(
                kf , kn , ktarget , goal_sector
            ))
            res.append((kf , kn , ktarget , goal_sector))
            
            ind = j
            kn = j
            valley_size = 1
            while ind < len(histogram) and valleys[ind]:
                valley_size += 1
                ind += 1
            if valley_size >= self.smax:
                print("wide valley")
                kf = kn + self.smax
            else:
                print("tight valley")
                kf = ind - 1
            
            assert kf is not None
            goal_sector = (kf + kn) // 2
            print("kf : {} kn : {} ktarg : {} goal_sector : {}".format(
                kf , kn , ktarget , goal_sector
            ))
            res.append((kf , kn , ktarget , goal_sector))
            goal_angles = [self.normalize_angle(g[3] * self.angle_increment) for g in res]
            print('left valley {} right valley {}'.format(*goal_angles))
            # (kf , kn , ktarget , goal_sector) = res[0] if abs(abs(goal_angles[0] - rotation)) < abs(abs(goal_angles[1] - rotation)) else res[1] 
            # (kf , kn , ktarget , goal_sector) = random.choice(res)
            (kf , kn , ktarget , goal_sector) = res[0] if abs(res[0][0] - res[0][1]) > abs(res[1][0] - res[1][1]) else res[1]
            
            
                
        if goal_sector is None:
            self.threshold = min(histogram) + 1000
            goal_sector = robot_heading_sector
            print(f"new threshold : {self.threshold}")
            
       
        goal_angle = self.normalize_angle(goal_sector * self.angle_increment) 
        
        print("rotation : {} goal_angle : {}".format(rotation , goal_angle))
        # plt.plot(list(range(len(histogram))),histogram)
        # plt.axhline(y=self.threshold , color = 'r')
        # plt.axvline(x=goal_sector , label='goal sector' ,color ='b')
        # if kn is not None and kf is not None:
        #     plt.axvline(x=kn , label ='kn' , color = 'c')
        #     plt.axvline(x=kf , label = 'kf' , color = 'g')
        # # plt.axvline(x=robot_heading_sector + len(histcopy), label = 'robot heading sector' , color = 'r')
        # plt.axvline(x=ktarget , label = 'ktarget' , color = 'y')
        
        # plt.plot(list(range(len(histcopy)*3)), np.array(histcopy.tolist() + histcopy.tolist() + histcopy.tolist()))
        # plt.show()
        
        tw_msg = Twist()
        
        last_angle = rotation
        
        first_angle = rotation
        
        turn_angle = 0
        
        tw_msg.angular.z = self.angular_speed * ( goal_angle - rotation) / abs(goal_angle - rotation)
        
        while abs(turn_angle) < abs(first_angle - goal_angle) and not rospy.is_shutdown():
            
            self.cmd_vel.publish(tw_msg)
            
            self.r.sleep()
            
            position , rotation = self.get_odom()
            
            delta_angle = self.normalize_angle(rotation - last_angle)
            
            turn_angle += delta_angle
            last_angle = rotation
            
        print("reached rotation is {}".format(self.get_odom()[1]))
        tw_msg = Twist()
        self.cmd_vel.publish(tw_msg)
        rospy.sleep(2)
        tw_msg = Twist()
        tw_msg.linear.x = self.linear_speed
        self.cmd_vel.publish(tw_msg)
        
        self.state = MOVE
        self.end_rotation = rospy.get_rostime()
        
        
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
