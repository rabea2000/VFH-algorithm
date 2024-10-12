#!/usr/bin/python3
import math
import numpy as np
from math import pi
import tf

import rospy
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import Pose
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point, Quaternion

class DetectObstacle:
    def __init__(self):
        
        self.GRIDSIZE = 135
        self.WINDOW = 25
        self.unit = 0.2
        self.ID = 0
        
        self.map_numpy = np.zeros((self.GRIDSIZE,self.GRIDSIZE), dtype=np.int8)
        rospy.init_node("ObstacleDetector" , anonymous=False)
        
        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.callback_laser_scan)
        self.window_pub = rospy.Publisher("/window_data", OccupancyGrid, queue_size=10)
      
        self.laser_flag = False
        

        self.rate = rospy.Rate(20)
 
        
        
        rospy.sleep(2)
        
    



    def callback_laser_scan(self, msg):
        if not self.laser_flag:
            self.laser_scan = msg
            self.laser_flag = True

    def update_grid(self):

        
        odom_data =  rospy.wait_for_message("/odom", Odometry)
        pos = odom_data.pose.pose.position       
        rot = self.get_current_heading(odom_data)
        

        
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
                x = int((range_scan * math.cos(angle) + position["x"]) / self.unit) + int((self.GRIDSIZE - 1) / 2)
                y = int((range_scan * math.sin(angle) + position["y"]) / self.unit) + int((self.GRIDSIZE - 1) / 2)
                self.map_numpy[y,x] = min(100, self.map_numpy[y,x] + 1)
        
        self.ID += 1

        robot_x = int(position["x"] / self.unit) + int((self.GRIDSIZE - 1) / 2)
        robot_y = int(position["y"] / self.unit) + int((self.GRIDSIZE - 1) / 2)
        
        half_window = int((self.WINDOW - 1) / 2)
        map_window_data = self.map_numpy[robot_y - half_window :robot_y + half_window + 1,robot_x - half_window :robot_x + half_window + 1]
        print('map')
        
        map_window_data = np.rot90(map_window_data , k =2)
        map_window_data = map_window_data[:, ::-1]         
        

        
        map_window = OccupancyGrid()
        map_window.header.seq = self.ID
        map_window.header.frame_id = "window_map"
        map_window.info.map_load_time = rospy.get_rostime()
        map_window.info.resolution = self.unit
        map_window.info.width = self.WINDOW
        map_window.info.height = self.WINDOW

        map_window.data = map_window_data.reshape(-1).tolist()
        self.window_pub.publish(map_window)
        
        

        self.laser_flag = False



        
    def get_current_heading(self, msg: Odometry):
        orientation = msg.pose.pose.orientation
        roll, pitch, yaw = tf.transformations.euler_from_quaternion((
            orientation.x ,orientation.y ,orientation.z ,orientation.w
        )) 
        return float("{:.2f}".format(yaw))


if __name__ == "__main__":
    obstacle_detector = DetectObstacle()
    while not rospy.is_shutdown():
        if obstacle_detector.laser_flag:
            obstacle_detector.update_grid()
            obstacle_detector.rate.sleep()





