#!/usr/bin/env python3

import re
import rospy
import argparse
import numpy as np
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist, Point
import math
from nav_msgs.msg import Path, Odometry
import os
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math

import warnings



class ReachTrack():
    ahead_time = 0.5 #in seconds
    radius =0.02
    ahead_dist = 0.2
    node_cnt=0
    first_time = True
    node_radius = 0.1
    got_path = False

    def path_cb(self, data):
        self.path_arr = []
        for elem in data.poses:
            point = [elem.pose.position.x, elem.pose.position.y]
            self.path_arr.append(point)
        self.got_path = True
        rospy.Subscriber("/odom", Odometry, self.odom_cb)
        rospy.spin()

    def odom_cb(self, data):
        self.pose = data.pose.pose
        self.twist = data.twist.twist
        self.vel = self.twist.linear.x
        self.look_ahead()
    
    def look_ahead(self):
        self.new_position = Point()
        self.new_position = self.pose.position
        orientation_q = self.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion (orientation_list)
        # self.new_yaw = self.yaw + self.twist.angular.z * self.ahead_time
        # self.new_position.x += self.twist.linear.x*self.ahead_time*math.cos(self.new_yaw)
        # self.new_position.y += self.twist.linear.x*self.ahead_time*math.sin(self.new_yaw)
        if self.first_time:
            self.line_segment(self.new_position)
            self.first_time = False
        else:
            self.continue_path(self.new_position)

    def line_segment(self, pt):
        i=0
        min_d = 10000
        self.pt = np.array([pt.x, pt.y])

        while i<len(self.path_arr)-1:

            start_p = np.copy(self.path_arr[i])
            end_p = np.copy(self.path_arr[i+1])
            v1 = end_p -start_p
            v2 = self.pt - start_p
            vs = v1 / np.linalg.norm(v1)
            ve = v2 / np.linalg.norm(v2)
            theta =  math.acos(abs(np.dot(vs,ve)))
            short_d = abs(np.linalg.norm(v2) * math.sin(theta))
            un_short_d = np.linalg.norm(v2) * math.sin(theta)
            if short_d<min_d:
                min_d = short_d
                min_i = i    
                un_d = un_short_d    
                self.theta =theta

            i=i+1
        
        start_p = np.copy(self.path_arr[min_i])
        end_p = np.copy(self.path_arr[min_i+1])
        self.cte = un_d
        v1 = end_p -start_p
        v2 = self.pt - start_p
        vs = v1 / np.linalg.norm(v1)
        self.projected_pt = start_p
        if np.dot(v1,v2)<0:
            self.projected_pt += vs*(-np.linalg.norm(v2)*math.cos(self.theta)+self.ahead_dist)
        else:
            self.projected_pt += vs*(np.linalg.norm(v2)*math.cos(self.theta)+self.ahead_dist)
        # print(self.projected_pt)
        # new_pt =np.array([2,2])
        # seek = Seek(self.pose.position, self.yaw, new_pt)
        self.cnt = min_i
        seek = Seek(self.pose.position, self.yaw, self.projected_pt, self.cte, self.vel)


    def continue_path(self, pt):
        self.pt = np.array([pt.x, pt.y])
        if self.node_cnt== len(self.path_arr)-1:
            rospy.signal_shutdown("Path Complete")
        start_p = np.copy(self.path_arr[self.node_cnt])
        end_p = np.copy(self.path_arr[self.node_cnt+1])
        v1 = end_p -start_p
        v2 = self.pt - start_p
        vs = v1 / np.linalg.norm(v1)
        ve = v2 / np.linalg.norm(v2)
        theta =  math.acos(abs(np.dot(vs,ve)))
        short_d = abs(np.linalg.norm(v2) * math.sin(theta))
        un_short_d = np.linalg.norm(v2) * math.sin(theta)
        self.projected_pt = start_p
        if np.dot(v1,v2)<0:
            self.incr_pt = vs*(-np.linalg.norm(v2)*math.cos(self.theta)+self.ahead_dist)
        else:
            self.incr_pt = vs*(np.linalg.norm(v2)*math.cos(self.theta)+self.ahead_dist)
        
        if self.calc_dist(end_p, self.pt)<self.node_radius: 
            self.node_cnt+=1
            if self.node_cnt == len(self.path_arr)-1:
                rospy.signal_shutdown("Path Complete")
            print("Path almost complete", self.node_cnt)
            start_p = np.copy(self.path_arr[self.node_cnt])
            end_p = np.copy(self.path_arr[self.node_cnt+1])
            v1 = end_p -start_p
            vs = v1 / np.linalg.norm(v1)
            ve = v2 / np.linalg.norm(v2)
            theta =  math.acos(abs(np.dot(vs,ve)))
            un_short_d = np.linalg.norm(v2) * math.sin(theta)
            short_d = abs(np.linalg.norm(v2) * math.sin(theta))
            self.projected_pt = start_p + vs*self.ahead_dist
        else:
            self.projected_pt += self.incr_pt
        print(self.node_cnt, end_p)
        self.cte = un_short_d

        seek = Seek(self.pose.position, self.yaw, self.projected_pt, self.cte, self.vel)


    def calc_dist(self, p1, p2):
        return math.sqrt((p1[0]-p2[0])**2+(p2[1]-p1[1])**2)

    def __init__(self):
        print("Follow Path node started") 
        # rospy.Subscriber("/odom", Odometry, self.odom_cb)
        rospy.Subscriber("/path", Path, self.path_cb)
        rospy.spin()


class Seek(object):
    max_vel = 0.8
    max_vel_dist = 1.5

    def reach(self):
        self.cmd_v = Twist()
        print("Heer")
        self.cmd_v.linear.y=0
        self.cmd_v.linear.z=0
        self.cmd_v.angular.y=0
        self.cmd_v.angular.z=0
        
        # self.cmd_v.linear.x = 0.2
        self.cmd_v.angular.z = self.steer()
        self.cmd_v.linear.x = self.linear_vel() 

        # print("At ", self.position,"To", self.target, "Current steer", self.cmd_v.angular.z)
        self.pub_vel.publish(self.cmd_v)

    def linear_vel(self):
        x1 = self.position.x
        y1 = self.position.y
        x2 =self.target[0]
        y2 = self.target[1]

        dist = math.sqrt((x1-x2)**2 + (y1-y2)**2)
        print(self.max_vel*np.clip(dist, 0, self.max_vel_dist)/self.max_vel_dist)

        return self.max_vel*np.clip(dist, 0, self.max_vel_dist)/self.max_vel_dist
    
    
    def steer(self): 
        req_steer = math.atan2(self.target[1]-self.position.y,self.target[0]-self.position.x) -self.yaw
        k_e = 0.25

        if req_steer > np.pi:
            req_steer -= 2 * np.pi
        if req_steer < - np.pi:
            req_steer += 2 * np.pi
        if req_steer > 0:
            self.cte = abs(self.cte)
        else:
            self.cte = - abs(self.cte)
        self.cte_steer = np.arctan(k_e * self.cte / self.v)


        self.theta_err = req_steer + self.cte_steer
        if self.theta_err>np.pi:
            self.theta_err = self.theta_err - 2*np.pi 
        elif self.theta_err<-np.pi:
            self.theta_err = self.theta_err + 2*np.pi 
    
        print("Here")
        return np.clip(self.theta_err, -1.57, 1.57)


    def __init__(self, curr_position, yaw, target, cte, vel, no_steer=False):
        # print("Seeking")
        self.target = target
        self.position = curr_position
        self.no_steer = no_steer
        self.yaw = yaw
        self.pub_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.cte = cte
        self.v = vel
        self.reach()

if __name__== "__main__":
    rospy.init_node("follow_path",anonymous=False)
    ld = ReachTrack()