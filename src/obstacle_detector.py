#!/usr/bin/python3

#import rospy library for ROS activities
from numpy.lib.function_base import select
import rospy

# Import the Odometry message
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import Pose
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point, Quaternion

#import mathematics libraries 
import math
import numpy as np
from math import pi
import tf
class ObstacleDetector:
    def __init__(self):
        
        self.GRIDSIZE = 115
        self.WINDOW = 21
        self.unit = 0.2
        
        self.ID = 0
        
        self.map_numpy = np.zeros((self.GRIDSIZE,self.GRIDSIZE), dtype=np.int8)
        
        # Initiate a named node
        rospy.init_node("ObstacleDetector" , anonymous=False)
        
        # Subscribe to topic /odom published by the robot base
        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.callback_laser_scan)
        self.window_pub = rospy.Publisher("/window_data", OccupancyGrid, queue_size=1)
        self.grid_pub = rospy.Publisher("grid_data", OccupancyGrid, queue_size=10)
        self.laser_flag = False
        
        self.obstacle_window = 5

        self.rate = rospy.Rate(1)
        
        self.base_frame = '/base_link'
        # The odom frame is usually just /odom
        self.odom_frame = '/odom'
        # Initialize the tf listener
        self.tf_listener = tf.TransformListener()
        
        rospy.sleep(2)
        
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
                    "Cannot find transform between /odom and /base_link or /base_footprint in obstacle detector")
                rospy.signal_shutdown("tf Exception")

        
        
    def quat_to_angle(self, quat):
        return tf.transformations.euler_from_quaternion((quat.x, quat.y, quat.z, quat.w))[2]

    def get_odom(self):
        # Get the current transform between the odom and base frames

        try:
            (trans, rot) = self.tf_listener.lookupTransform(
                self.odom_frame, self.base_frame, rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return

        return (Point(*trans), self.quat_to_angle(Quaternion(*rot)))


    def callback_laser_scan(self, msg):
        if not self.laser_flag:
            self.laser_scan = msg
            self.laser_flag = True

    def update_grid(self):
        
        pos , rot = self.get_odom()
        
        position = {
            'x':pos.x,
            'y':pos.y,
            'theta':rot
        }
        
        
        num_angles = round(
            (self.laser_scan.angle_max - self.laser_scan.angle_min)
            / self.laser_scan.angle_increment
        )
        for i in range(num_angles):
            range_scan = self.laser_scan.ranges[i]
            if not math.isinf(range_scan) and range_scan > self.laser_scan.range_min and range_scan < self.laser_scan.range_max:
                angle = i * self.laser_scan.angle_increment + self.laser_scan.angle_min + position["theta"]
                x = int((range_scan * math.cos(angle) + position["x"]) / self.unit) + int((self.GRIDSIZE + 1) / 2)
                y = int((range_scan * math.sin(angle) + position["y"]) / self.unit) + int((self.GRIDSIZE + 1) / 2)
                self.map_numpy[x,y] = min(100, self.map_numpy[x,y] + 1)
        
        robot_x = int(position["x"] / self.unit) + int((self.GRIDSIZE + 1) / 2)
        robot_y = int(position["y"] / self.unit) + int((self.GRIDSIZE + 1) / 2)
        
        half_window = int((self.WINDOW - 1) / 2)
        map_window_data = self.map_numpy[robot_x - half_window :robot_x + half_window + 1, robot_y - half_window :robot_y + half_window + 1]
        
        self.ID += 1
        
        map_window = OccupancyGrid()
        # header
        map_window.header.seq = self.ID
        map_window.header.frame_id = "map"
        # info
        map_window.info.map_load_time = rospy.get_rostime()
        map_window.info.resolution = 0.2
        map_window.info.width = self.WINDOW
        map_window.info.height = self.WINDOW
        map_window.info.origin = self.grid_to_pose(robot_x - half_window, robot_y - half_window)
        # data
        map_window.data = map_window_data.reshape(-1).tolist()
        obstacle_detected = len([i for i in self.laser_scan.ranges[:self.obstacle_window] + self.laser_scan.ranges[-self.obstacle_window:] if (not math.isinf(i)) and i < 1.5]) > 0
        # print(obstacle_detected)
        if obstacle_detected: 
            self.window_pub.publish(map_window)
            rospy.sleep(10)

        
        map_grid = OccupancyGrid()
        # header
        map_grid.header.seq = self.ID
        map_grid.header.frame_id = "map"
        # info
        map_grid.info.map_load_time = rospy.get_rostime()
        map_grid.info.resolution = 0.2
        map_grid.info.width = self.GRIDSIZE
        map_grid.info.height = self.GRIDSIZE
        map_grid.info.origin = self.grid_to_pose(0, 0)
        # data
        map_grid.data = self.map_numpy.reshape(-1).tolist()
        self.grid_pub.publish(map_grid)
        
        self.laser_flag = False


    def grid_to_pose(self, x_grid , y_grid):
        mid_grid = (self.GRIDSIZE - 1) / 2
        output = Pose()
        output.position.x = (x_grid - mid_grid) * self.unit
        output.position.y = (y_grid - mid_grid) * self.unit
        return output




if __name__ == "__main__":
    obstacle_detector = ObstacleDetector()
    while not rospy.is_shutdown():
        if obstacle_detector.laser_flag:
            obstacle_detector.update_grid()
            obstacle_detector.rate.sleep()
