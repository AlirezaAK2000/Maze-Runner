#!/usr/bin/python3

import rospy

# Import the Odometry message
from nav_msgs.msg import Odometry, OccupancyGrid

from sensor_msgs import LaserScan

import math
import numpy as np


class ObstacleDetector:
    def __init__(self):
        self.GRIDSIZE = 105
        self.unit = 0.2
        self.map_numpy = np.zeroes((GRIDSIZE,GRIDSIZE), dtype=np.int8)
        # Initiate a named node
        rospy.init_node("Obstacle_detector", anonymous=True)
        # Subscribe to topic /odom published by the robot base
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.callback_odometry)
        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.callback_laser_scan)
        self.grid_pub = rospy.Publisher("/occupancy_grid", OccupancyGrid, queue_size=10)

        self.laser_flag = False
        self.odometry_flag = False

        self.rate = rospy.Rate(5)

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
            range = self.laser_scan.ranges[i]
            if not math.isinf(range) and range > self.laser_scan.range_min and range > self.laser_scan.range_max:
                angle = i * self.laser_scan.angle_increment + self.laser_scan.angle_min + self.position["theta"]
                x = int((range * math.sin(angle) + self.position["x"]) / self.unit) + (self.GRIDSIZE + 1) / 2
                y = int((range * math.cos(angle) + self.position["y"]) / self.unit) + (self.GRIDSIZE + 1) / 2
                self.map_numpy[x,y] += 1
                map_grid = OccupancyGrid()
                map_grid.info.width = self.GRIDSIZE
                map_grid.info.height = self.GRIDSIZE
                map_grid.data = map_numpy.tolist()
                
                
                
                
            
            
            


if __name__ == "__main__":
    obstacle_detector = ObstacleDetector()
    while not rospy.is_shutdown():
        if obstacle_detector.laser_flag and obstacle_detector.odometry_flag:
            obstacle_detector.update_grid()
    rospy.spin()
