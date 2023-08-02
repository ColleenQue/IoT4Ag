#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
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
        self.current_position_x,self.current_position_y=None,None



        #deine variables
        self.area = None
        self.low = False
        self.current_goal=None
        self.around_points = []
        self.around_index = 0
        self.glitching = False

        #partial low level ones
        self.low_goal = None
        
        #subcribers
        self.loc = rospy.Subscriber("/pixel_loc",  PixelLocationList, self.pixel_loc_callback, queue_size=1) #for current relationship with target
        self.area_sub = rospy.Subscriber("/contour_size",  ContourSizeList, self.area_callback, queue_size=1) #for current relationship with target
        self.sub  = rospy.Subscriber('/gazebo/model_states', ModelStates,self. model_states_callback,queue_size=1) #for current position and orientation

        #publishers
        self.pub = rospy.Publisher('/jackal_velocity_controller/cmd_vel', Twist, queue_size=1)


        rospy.spin()

    def model_states_callback(self, msg):    
    
        #print("subuscribing model states------------------------------------")
        last_model_pose = msg.pose[-1]

        #get current position
        self.current_position_x = last_model_pose.position.x
        self.current_position_y = last_model_pose.position.y
        #print(self.current_position_x , self.current_position_y)
        #get current angle
        self.quaternion = last_model_pose.orientation
        self.current_angle = self.quaternion_to_radians()


        
        #only go if not in low level planner
        if not self.low:
            self.perform_action()

        

    def pixel_loc_callback(self,msg):
        #print("subuscribing pixel locations------------------------------------")

        my_pixel_list = msg.pixel_location_list
        if len(my_pixel_list)>0:
            self.pixel_coordinates = msg.pixel_location_list[0].pixel_location_xy[0]
            #print("current coordinates: " ,self.pixel_coordinates)


    def area_callback(self,msg):
        
    
        contour_area_list = msg.contour_size_list
        if len(contour_area_list)>0:
            self.area = (msg.contour_size_list[0].contour_size)

            #print("detecting biggest area",self.area)
            #move only if contour size is in range and has pixel location

            if self.area>=6000 or len(self.around_points)>0:
                #time to move around
                self.move_around_action()
                return
            
            if self.glitching or (self.area>=500 and self.area<=6000):
                
                #if self.area>=4000:
                #self.lastCoordinates = []
                self.low  = True

                if self.pixel_coordinates:
                    self.low_level_action()
                    return
        #return to high level no contour size is in range


        if len(self.around_points)>0:
            #time to move around
            self.move_around_action()
            return

        self.low=False

    def move_around(self):
        #identifies the way points for moving around a target object
        #calculate the goal angle
        #step 1
        self.goal_angle = self.current_angle + math.pi/4
        # if goal_angle >  math.pi:
        #     goal_angle = goal_angle - 3.14
        i=0
        self.around_points = []
        curr_x,curr_y = self.current_position_x,self.current_position_y

        while i<3:

            delta_x = math.cos(self.goal_angle) *2
            delta_y = math.sin(self.goal_angle) *2
            curr_x = curr_x+delta_x
            curr_y = curr_y + delta_y

            self.around_points.append([curr_x,curr_y])

            self.goal_angle -= math.pi/2
    
            i+=1
        print(self.around_points)
        

    def move_around_action(self):

        print("moving around")

        if not self.low_goal:
            self.move_around()
            self.around_index=0
            self.low_goal = self.around_points[self.around_index]


        self.goal_position_x = self.low_goal[0]
        self.goal_position_y = self.low_goal[1]

        self.distance_x = self.goal_position_x  - self.current_position_x
        self.distance_y = self.goal_position_y- self.current_position_y
        distance = self.Pythagorean()



        if distance < 0.2:
            #go to the next goal
            self.around_index += 1
            if self.around_index >= len(self.around_points):
                #exit
                self.low=False
                self.around_points=[]
                self.around_index = 0
                return

            self.low_goal = self.around_points[self.around_index]

        print("current index: ", self.around_index)
        print("current goal: ", self.low_goal)

        ##### angle    
        #calculate desired goal angle based on distance_x and distance_y
        self.goal_angle = math.atan2(self.distance_y,self.distance_x)
        angle = self.goal_angle - self.current_angle


        #edge case where self.goal_angle and current_angle are closer to 3.14
        # #that is when abs(goal) + abs(current) > pi and goal*current <0
        if abs(self.current_angle)+abs(self.goal_angle)>math.pi and self.current_angle*self.goal_angle<0:
            angle = -1/angle


        #### assign angular and linear velocity so the robot moves
        cmd_vel = Twist()
        if abs(angle) <= 0.1:
            #move only if angle is correct
            cmd_vel.linear.x = distance/2
        else:
            cmd_vel.linear.x = 0

        cmd_vel.angular.z = angle*2

        #move
        self.pub.publish(cmd_vel)

    
    def low_level_action(self):
        #print("low level action")
        if self.area<600:
            #probably glitchingl
            self.glitching = True
        else:
            self.glitching = False



        #print(self.pixel_coordinattces)

        cmd_vel = Twist()

        if self.pixel_coordinates >200 and self.pixel_coordinates<400:
            #move forward
            cmd_vel.angular.z = 0
            cmd_vel.linear.x= 1000/self.area
            #cmd_vel.linear.x= 100

        elif self.pixel_coordinates > 300:
            #turn right
            cmd_vel.angular.z = -self.pixel_coordinates/300
            cmd_vel.linear.x=0
        else:
            #turn left, the smaller it is, the larger the angular velocity
            cmd_vel.angular.z = self.pixel_coordinates/300
            cmd_vel.linear.x=0

        
        #move
        self.pub.publish(cmd_vel)


    def perform_action(self):
        #print("high leveling")
        if not self.current_goal: 
            #for the trajectory
            bottom_left = [-9.9,-9.9]
            top_right = [9.9,9.9]
            diagonal_corners = [bottom_left, top_right]
            #print(diagonal_corners)
            num_polygons = 5
            direction = "horizontal"
            self.waypoints = self.get_lawn_mower_pts(diagonal_corners,num_polygons,direction)
            #print(self.waypoints)
            #[x,y]
            self.current_index = 0
            self.current_goal = self.waypoints[0]

        #print(self.current_goal)
            


        self.goal_position_x = self.current_goal[0]
        self.goal_position_y = self.current_goal[1]

        self.distance_x = self.goal_position_x  - self.current_position_x
        self.distance_y = self.goal_position_y- self.current_position_y
        distance = self.Pythagorean()

        if distance < 0.2:
            #go to the next goal
            self.current_index += 1
            if self.current_index>= len(self.waypoints):
                #reset & reverse the list
                self.waypoints.reverse()
                self.current_index = 0
            self.current_goal = self.waypoints[self.current_index]


        ##### angle    
        #calculate desired goal angle based on distance_x and distance_y
        self.goal_angle = math.atan2(self.distance_y,self.distance_x)
        angle = self.goal_angle - self.current_angle


        #edge case where self.goal_angle and current_angle are closer to 3.14
        # #that is when abs(goal) + abs(current) > pi and goal*current <0
        if abs(self.current_angle)+abs(self.goal_angle)>math.pi and self.current_angle*self.goal_angle<0:
            angle = -1/angle


        #### assign angular and linear velocity so the robot moves
        cmd_vel = Twist()
        if abs(angle) <= 0.1:
            #move only if angle is correct
            cmd_vel.linear.x = distance*2
        else:
            cmd_vel.linear.x = 0

        cmd_vel.angular.z = angle*2

        #move
        self.pub.publish(cmd_vel)


    def quaternion_to_radians(self):
        # Extract the x, y, and z components of the quaternion
        x = self.quaternion.x
        y = self.quaternion.y
        z = self.quaternion.z
        w = self.quaternion.w

        yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))

        return yaw
    

    def Pythagorean(self):
        # Calculate the differences in x and y coordinates
        
        # Calculate the square of the differences
        dx_squared = self.distance_x  ** 2
        dy_squared = self.distance_y ** 2

        # Calculate the sum of the squares
        sum_of_squares = dx_squared + dy_squared

        # Calculate the square root of the sum of squares
        distance = math.sqrt(sum_of_squares)

        return distance
    


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

        


        




