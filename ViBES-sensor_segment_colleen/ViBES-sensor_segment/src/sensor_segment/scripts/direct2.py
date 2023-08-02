#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.msg import ModelStates
import numpy as np
import math
from itertools import product
import cv2 as cv
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge
from sensor_segment.msg import PixelLocationList
from sensor_segment.msg import ContourSizeList
# importing the required module
import matplotlib.pyplot as plt
import signal
import sys

class Direct:
    def __init__(self):
        print("[Direct Node] Initializing ...")
        rospy.init_node("square_move", anonymous=False)


        self.rate = rospy.Rate(1)

        #set parameters
        bottom_left = [-9.9,-9.9]
        top_right = [9.9,9.9]
        diagonal_corners = [bottom_left, top_right]
        num_polygons = 5
        direction = "horizontal"

        self.waypoints = self.get_lawn_mower_pts(diagonal_corners,num_polygons,direction)

        #deine variables
        self.current_position_x,self.current_position_y=None,None
        
        self.current_goal_x,self.current_goal_y = self.waypoints[0][0],self.waypoints[0][1]

        self.published = -1

        #for graphing
        self.pixel_coordinates=None
        self.trajectory_x = []
        self.trajectory_y = []

        
        #subcribers
        self.sub  = rospy.Subscriber('/gazebo/model_states', ModelStates,self. model_states_callback,queue_size=1) #for current position and orientation

        #publishers
        self.pub = rospy.Publisher('/move_base/current_goal', PoseStamped, queue_size=1)


        rospy.spin()

    def model_states_callback(self, msg):    
    
        #print("subuscribing model states------------------------------------")
        last_model_pose = msg.pose[-1]

        #get current position
        self.current_position_x = last_model_pose.position.x
        self.current_position_y = last_model_pose.position.y

        #publish the new goal when we approximately reach the current goal

        if self.published == -1:
            self.current_index = 0
        

        if self.current_position_x < self.current_goal_x+0.1 and self.current_position_x > self.current_goal_x-1 and self.current_position_y < self.current_goal_y+0.1 and self.current_position_y > self.current_goal_y-1:
            #goal is reached = move to the next target
            self.current_index+=1

        self.current_goal_x = self.waypoints[self.current_index][0]
        self.current_goal_y = self.waypoints[self.current_index][1]
        self.publish_goal()
        if self.current_index > self.published:
            self.publish_goal()
            self.published = self.current_index
            
        
    def publish_goal(self):
        #publish the next goal
        cmd_goal = PoseStamped()


        cmd_goal.header.frame_id = 'map'
        #intialize vs. update

        cmd_goal.pose.position.x = -9.9 # self.waypoints[self.current_index][0]
        cmd_goal.pose.position.y = -9.9#self.waypoints[self.current_index][1]
        
        print(cmd_goal)
        self.pub.publish(cmd_goal)
        print("published")

        
    def get_lawn_mower_pts(self,diagonal_corners, num_polygons, direction):
        # align diagonal_corners with regular (x-y) cartesian co-ordinates
        # i.e. treat diagonal_corners[0] as bottom left (start), diagonal_corners[1] as top right
        # (and end for even number of polygons or penultemate if for odd number of polygons)
        # then we have either vertical columns or horizontal columns
        # get the ranges in x and y direction
        x_range = abs(diagonal_corners[0][0] - diagonal_corners[1][0])
        y_range = abs(diagonal_corners[0][1] - diagonal_corners[1][1])
        # get number of waypoints
        num_pts = 4 + (num_polygons - 1) * 2
        # see points in pairs of same y values, exluding the first
        points = [0] * num_pts
        points[0] = diagonal_corners[0]
        # fill in all the rest of points
        if direction == 'vertical':
            for i in range(0, num_pts, 2):
                points[i]     = (diagonal_corners[0][0] + (i/2) * (x_range/num_polygons), diagonal_corners[0][1] + (i % 1) * y_range)
                points[i + 1] = (diagonal_corners[0][0] + (i/2) * (x_range/num_polygons), diagonal_corners[0][1] + (i % 2 + 1) * y_range)
        elif direction == 'horizontal':
            for i in range(0, num_pts, 2):
                points[i]     = (diagonal_corners[0][0] + (i % 1) * x_range, diagonal_corners[0][1] + (i/2) * (y_range/num_polygons))
                points[i + 1] = (diagonal_corners[0][0] + (i % 2 + 1) * x_range, diagonal_corners[0][1] + (i/2) * (y_range/num_polygons))
        # flip every other pair for lawn mower pattern
        for i, pt in enumerate(points):
            if (i + 1) % 4 == 0:
                points[i], points[i-1] = points[i-1], points[i]
        return points


    

if __name__ == "__main__":

    node = Direct()
    
    while not rospy.is_shutdown():
        node.rate.sleep()

        


        




