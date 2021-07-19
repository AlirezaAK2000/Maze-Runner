#!/usr/bin/python3

#import rospy library for ROS activities
import rospy

# Import the Odometry message
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import Pose
from sensor_msgs.msg import LaserScan

#import mathematics libraries 
import math
import numpy as np


class ObstacleDetector:
    def __init__(self):
        
        self.GRIDSIZE = 115
        self.WINDOW = 21
        self.unit = 0.2
        
        self.ID = 0
        
        self.map_numpy = np.zeros((self.GRIDSIZE,self.GRIDSIZE), dtype=np.int8)
        
        # Initiate a named node
        rospy.init_node("ObstacleDetector")
        
        # Subscribe to topic /odom published by the robot base
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.callback_odometry)
        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.callback_laser_scan)
        self.window_pub = rospy.Publisher("/window_data", OccupancyGrid, queue_size=10)
        self.grid_pub = rospy.Publisher("grid_data", OccupancyGrid, queue_size=10)
        

        self.laser_flag = False
        self.odometry_flag = False

        self.rate = rospy.Rate(1)

    def callback_odometry(self, msg):
        if not self.odometry_flag:
            self.position = {
                "x": msg.pose.pose.position.x,
                "y": msg.pose.pose.position.y,
                "theta": msg.pose.pose.orientation.x
            }

            self.odometry_flag = True

    def callback_laser_scan(self, msg):
        if not self.laser_flag:
            self.laser_scan = msg
            self.laser_flag = True

    def update_grid(self):
        num_angles = round(
            (self.laser_scan.angle_max - self.laser_scan.angle_min)
            / self.laser_scan.angle_increment
        )
        for i in range(num_angles):
            range_scan = self.laser_scan.ranges[i]
            if not math.isinf(range_scan) and range_scan > self.laser_scan.range_min and range_scan < self.laser_scan.range_max:
                angle = i * self.laser_scan.angle_increment + self.laser_scan.angle_min + self.position["theta"]
                x = int((range_scan * math.sin(angle) + self.position["x"]) / self.unit) + int((self.GRIDSIZE + 1) / 2)
                y = int((range_scan * math.cos(angle) + self.position["y"]) / self.unit) + int((self.GRIDSIZE + 1) / 2)
                self.map_numpy[x,y] = min(100, self.map_numpy[x,y] + 1)
        
        robot_x = int(self.position["x"] / self.unit) + int((self.GRIDSIZE + 1) / 2)
        robot_y = int(self.position["y"] / self.unit) + int((self.GRIDSIZE + 1) / 2)
        
        half_window = int((self.WINDOW - 1) / 2)
        map_window_data = self.map_numpy[robot_x - half_window :robot_x + half_window + 1, robot_y - half_window :robot_y + half_window + 1]
        
        self.ID += 1
        
        map_window = OccupancyGrid()
        # header
        map_window.header.seq = self.ID
        map_window.header.frame_id = "map"
        # info
        map_window.info.map_load_time = rospy.get_rostime()
        map_window.info.width = self.WINDOW
        map_window.info.height = self.WINDOW
        map_window.info.origin = self.grid_to_pose(robot_x, robot_y)
        # data
        map_window.data = map_window_data.reshape(self.WINDOW * self.WINDOW).tolist()
        if not math.isinf(self.laser_scan.ranges[len(self.laser_scan.ranges) // 2]): 
            self.window_pub.publish(map_window)

        
        map_grid = OccupancyGrid()
        # header
        map_grid.header.seq = self.ID
        map_grid.header.frame_id = "map"
        # info
        map_grid.info.map_load_time = rospy.get_rostime()
        map_grid.info.width = self.GRIDSIZE
        map_grid.info.height = self.GRIDSIZE
        map_grid.info.origin = self.grid_to_pose(0, 0)
        # data
        map_grid.data = self.map_numpy.reshape(self.GRIDSIZE * self.GRIDSIZE).tolist()
        self.grid_pub.publish(map_grid)
        
        self.laser_flag = False
        self.odometry_flag = False


    def grid_to_pose(self, x_grid , y_grid):
        mid_grid = (self.GRIDSIZE + 1) / 2
        output = Pose()
        output.position.x = (x_grid - mid_grid) * self.unit
        output.position.y = (y_grid - mid_grid) * self.unit
        return output




if __name__ == "__main__":
    obstacle_detector = ObstacleDetector()
    while not rospy.is_shutdown():
        if obstacle_detector.laser_flag and obstacle_detector.odometry_flag:
            obstacle_detector.update_grid()
            obstacle_detector.rate.sleep()
