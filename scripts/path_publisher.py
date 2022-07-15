#!/usr/bin/env python3

import rospy
import argparse
import numpy as np
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist, Point, Pose, PoseStamped
import math
from nav_msgs.msg import Path, Odometry
import os
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math

import warnings
import sys


class CreatePath(object):
    scale_ratio = 0.01
    shift_x = -176.42010812403808
    shift_y = -209.82127477522036
    is_neg_x = False
    is_neg_y = True
    

    def __init__(self):
        self.path_pub = rospy.Publisher("path", Path, queue_size=10)

        self.send_path()
    
    def send_path(self):
        path = Path()

        path.header.frame_id = "odom"


        args = rospy.myargv(argv=sys.argv)
        self.path_dir = args[1]
        file1 = open(self.path_dir + "/path.txt", 'r')
        # cnt=0
        # file1 = open("~/turtle_ws/src/turtle_pkg/config/path.txt", "r")
        Lines = file1.readlines()
        for line in Lines:
            # print(line)
            pose =  PoseStamped()
            x, y, z = line.split(",")
            x1 = float(x)
            y1 = float(y)

            pose.pose.position.z = 0.0
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = 0.0
            pose.pose.orientation.w = 1.0
            pose.pose.position.x = self.scale_ratio*(x1 - self.shift_x)
            pose.pose.position.y = -1 *self.scale_ratio * (y1 - self.shift_y)
            path.poses.append(pose)
        #     cnt+=1
        # print("Path Created",cnt)
        while not rospy.is_shutdown():
            self.path_pub.publish(path)
            rate = rospy.Rate(10)
            rate.sleep()


if __name__== "__main__":
    rospy.init_node("path_publisher",anonymous=False)
    ld = CreatePath()