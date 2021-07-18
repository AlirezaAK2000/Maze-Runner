#!/usr/bin/env python

import rospy

# Import the Odometry message
from nav_msgs.msg import Odometry ,Path

from geometry_msgs.msg import PoseStamped


# Import the Twist message
from geometry_msgs.msg import Twist

# TF allows to perform transformations between different coordinate frames
import tf

# For getting robot’s ground truth from Gazebo
from gazebo_msgs.srv import GetModelState

class PathPlanner():
    def __init__(self):
        
        # Initiate a named node
        rospy.init_node('obstacle_detector', anonymous=True)
        
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.callback_odometry)
        
        self.path_pub = rospy.Publisher('/path', Path, queue_size=10)
        
        
        # Subscribe to topic /change published by move_robot
        self.vel_change_sub = rospy.Subscriber('/change', Twist,
        self.callback_velocity_change)
        self.report_pose = False
        
        # subscribe to a service server, provided by the gazebo package to get
        
        # information about the state of the models present in the simulation
        print("Wait for service ....")
        rospy.wait_for_service("gazebo/get_model_state")
        try:
            print(" ... Got it!")
            self.get_ground_truth = rospy.ServiceProxy("gazebo/get_model_state", GetModelState)
            rospy.logdebug(f"service provided : {self.get_ground_truth}")
        except rospy.ServiceException:
            rospy.logerr("service proxy doen't provided!!!")
    
    
    def callback_velocity_change(self, msg):
        rospy.loginfo("Velocity has changed, now: %5.3f, %5.3f",msg.linear.x, msg.angular.z)
        rospy.sleep(0.75) 
        # to let the velocity being actuated and odometry updated
        self.report_pose = True
    
    def callback_odometry(self, msg):
        if self.report_pose == True:
            print("Position: (%5.2f, %5.2f, %5.2f)" % (msg.pose.pose.position.x,msg.pose.pose.position.y, msg.pose.pose.position.z))
            self.quaternion_to_euler(msg)
            print("Linear twist: (%5.2f, %5.2f, %5.2f)" % (msg.twist.twist.linear.x,msg.twist.twist.linear.y, msg.twist.twist.linear.z))
            print("Angular twist: (%5.2f, %5.2f, %5.2f)" % (msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z))
            print("Ground Truth: ", self.get_ground_truth("turtlebot3_waffle", "world"))
            self.report_pose = False
        
        # for displaying trajectory in rviz
        self.path.header = msg.header
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.path.poses.append(pose)
        self.path_pub.publish(self.path)
                
                
            
    def quaternion_to_euler(self, msg):
        quaternion = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quaternion)
        print("Roll: %5.2f Pitch: %5.2f Yaw: %5.2f" % (roll, pitch, yaw))


if __name__ == '__main__':
    pass