#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped, Point
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates

from sensor_segment.msg import PixelLocationList
from sensor_segment.msg import ContourSizeList
from gazebo_msgs.msg import LinkStates

import numpy as np
import math


class Waypoint:
    def __init__(self):
        print("[Waypoint Node] Initializing ...")
        rospy.init_node("Waypoint", anonymous=False)
        self.rate = rospy.Rate(1)


        #initialize variables
        self.pixel_coordinates=None
        self.current_position_x,self.current_position_y=None,None
        self.waypoints1=None
        self.waypoints= None
        self.current_index = 0
        self.current_goal=None
        self.low =False
        self.glitching = False

        self.store_targets = []
        self.store_robot = []

        self.next = False
        self.exists=False

        self.loc = rospy.Subscriber("/pixel_loc",  PixelLocationList, self.pixel_loc_callback, queue_size=1) #for current relationship with target
        self.area_sub = rospy.Subscriber("/contour_size",  ContourSizeList, self.area_callback, queue_size=1) #for current relationship with target
        self.sub  = rospy.Subscriber('/gazebo/model_states', ModelStates,self. model_states_callback,queue_size=1) #for current position and orientation
        self.sub1  = rospy.Subscriber('/gazebo/link_states', LinkStates,self. link_callback,queue_size=1) #for current position and orientation

        #publishers
        self.pub = rospy.Publisher('/jackal_velocity_controller/cmd_vel', Twist, queue_size=1)

        rospy.spin()


    def link_callback(self,msg):
        #print("subuscribing link states------------------------------------")

        if not self.low:
            self.perform_action()


    def model_states_callback(self, msg):    
    
        #print("subuscribing model states------------------------------------")
        last_model_pose = msg.pose[-1]
        # print("index",self.current_index)
        # print("self low: ",self.low)
        #get current position
        self.current_position_x = last_model_pose.position.x
        self.current_position_y = last_model_pose.position.y
        #print(self.current_position_x , self.current_position_y)
        #get current angle
        self.quaternion = last_model_pose.orientation
        self.current_angle = self.quaternion_to_radians()
        # print(self.current_position_x)
        # print(self.current_position_y)
        # print(self.current_angle)


    def pixel_loc_callback(self,msg):
        #prints current coordinates
        #print("subuscribing locations------------------------------------")
        my_pixel_list = msg.pixel_location_list
        if len(my_pixel_list)>0:
            self.pixel_coordinates = msg.pixel_location_list[0].pixel_location_xy[0]
            # print("current coordinates: " ,self.pixel_coordinates)

        
    
    def area_callback(self,msg):

        contour_area_list = msg.contour_size_list

        if len(contour_area_list)>0:
            self.area = (msg.contour_size_list[0].contour_size)

            #if self.glitching or (self.area>=1000 and self.area<=20000):
            if self.area>=1500 and self.area<=20000:
                #if self.area>=4000:
                #self.lastCoordinates = []
                self.low  = True

                if self.pixel_coordinates:
                    self.low_level_action()
            
                    return
        #return to high level no contour size is in range

        self.low=False



    def low_level_action(self):
        #make the robot travel towards the object...

        #face the red ball before moving
        cmd_vel = Twist()

        if self.pixel_coordinates >290 and self.pixel_coordinates<310:
            #move forward
            self.move_forward()
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



    def move_forward(self):
        print("robot coordinates", self.current_position_x, self.current_position_y)
        delta_x = math.cos(self.current_angle)*math.sqrt(30000/self.area) *1.3
        delta_y = math.sin(self.current_angle)*math.sqrt(30000/self.area) *1.3

        low_goal_x = self.current_position_x + delta_x
        low_goal_y = self.current_position_y + delta_y


        self.exists = False

        for x,y in self.store_targets:
            x_diff = abs(x-low_goal_x)
            y_diff = abs(y-low_goal_y)

            if self.Pythagorean(x_diff,y_diff)<4:
                self.exists = True
                self.low=True
                break

        if not self.exists:
            #document the targets
            self.store_targets.append([low_goal_x,low_goal_y])
            self.store_robot.append([self.current_position_x ,self.current_position_y ])
            
            #write to document
            f = open("output3.txt", "w")
            f.write("processed location " + str(self.store_targets))
            f.write('\n')
            f.write("robot location" + str(self.store_robot))
            f.close() 

            #move towards the goal...
            action_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
            action_client.wait_for_server()

            #generate goal points
            waypoints = self.circular_motion(low_goal_x,low_goal_y,8)
            print(waypoints)
            waypoints1 = []
            for x,y in waypoints:
                waypoints1.append(self.create_waypoint(x,y))


            for waypoint in waypoints1:
                goal = MoveBaseGoal()
                goal.target_pose = waypoint
                action_client.send_goal(goal)
                

                finished_within_time = action_client.wait_for_result(rospy.Duration(900.0))

                # Check the status of the action
                if finished_within_time:
                    state = action_client.get_state()
                    if state == actionlib.GoalStatus.SUCCEEDED:
                        rospy.loginfo('Navigation to the goal succeeded!')
                    elif state == actionlib.GoalStatus.PREEMPTED:
                        rospy.logwarn('Navigation goal was preempted.')
                    elif state == actionlib.GoalStatus.ABORTED:
                        rospy.logwarn('Navigation goal aborted.')
                    else:
                        rospy.logwarn('Navigation goal failed.')
                    
                    # Get the result of the action
                    result = action_client.get_result()
                    # Use the result data if needed
                    rospy.loginfo('Result: {}'.format(result))
                else:
                    # If the action did not complete within the specified timeout
                    rospy.logwarn('Navigation did not complete in time.')
        
        else:
            #move in high level
            self.perform_action()


    def circular_motion(self,start_x,start_y,stopping_points):
        self.goal_angle = self.current_angle + math.pi/stopping_points
        around_points = [[start_x,start_y]]
        i=0
        curr_x,curr_y = start_x,start_y
        while i<stopping_points:
            delta_x = math.cos(self.goal_angle) *2
            delta_y = math.sin(self.goal_angle) *2
            curr_x = curr_x+delta_x
            curr_y = curr_y + delta_y
            around_points.append([curr_x,curr_y])
            self.goal_angle -= math.pi/stopping_points *2
            i+=1
        
        return around_points


    
    def create_waypoint(self,x, y,qx=0,qy=0,qz=0,qw=1):
        waypoint = PoseStamped()
        waypoint.header.frame_id = "map"  # Set the frame_id to the correct reference frame
        waypoint.pose.position.x = x
        waypoint.pose.position.y = y
        waypoint.pose.position.z = 0
        
        qx, qy, qz, qw = self.normalize_quaternion(qx,qy,qz,qw)
        
        waypoint.pose.orientation.x = qx
        waypoint.pose.orientation.y = qy
        waypoint.pose.orientation.z = qz
        waypoint.pose.orientation.w = qw
        return waypoint
    
    

    def publish_waypoints(self):
        print("publishing way points")
        
        action_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        action_client.wait_for_server()


        self.current_goal = self.waypoints1[self.current_index]

        goal = MoveBaseGoal()
        goal.target_pose = self.waypoints1[self.current_index]
        print(goal.target_pose)
        action_client.send_goal(goal)
        # action_client.wait_for_result(rospy.Duration(10,0))
        # #rospy.sleep(rospy.Duration(10,0))
        # result = action_client.get_result()
        # print(result)
        

        finished_within_time = action_client.wait_for_result(rospy.Duration(90000.0))

        # Check the status of the action
        if finished_within_time:
            state = action_client.get_state()
            if state == actionlib.GoalStatus.SUCCEEDED:
                rospy.loginfo('Navigation to the goal succeeded!')
                self.current_index+=1
                if self.current_index>= len(self.waypoints1):
                    #reset & reverse the list
                    self.waypoints1.reverse()
                    self.current_index = 0
                return


            elif state == actionlib.GoalStatus.PREEMPTED:
                rospy.logwarn('Navigation goal was preempted.')
            elif state == actionlib.GoalStatus.ABORTED:
                rospy.logwarn('Navigation goal aborted.')
                # delta_x = math.cos(self.current_angle)*math.sqrt(30000/self.area) 
                # delta_y = math.sin(self.current_angle)*math.sqrt(30000/self.area)

                # low_goal_x = self.current_position_x + delta_x
                # low_goal_y = self.current_position_y + delta_y

                # self.custom_waypoint(low_goal_x,low_goal_y)
                self.current_index+=1
                if self.current_index>= len(self.waypoints1):
                    #reset & reverse the list
                    self.waypoints1.reverse()
                    self.current_index = 0
                
            else:
                rospy.logwarn('Navigation goal failed.')
            
            # Get the result of the action
            result = action_client.get_result()
            # Use the result data if needed
            rospy.loginfo('Result: {}'.format(result))
        else:
            # If the action did not complete within the specified timeout
            rospy.logwarn('Navigation did not complete in time.')


        current_goal_x = self.waypoints[self.current_index][0]
        current_goal_y =  self.waypoints[self.current_index][1]


        if abs(current_goal_x - self.current_position_x) <1.5 and abs(current_goal_y - self.current_position_y) <1.5:
            #increment
            self.current_index+=1
            if self.current_index>= len(self.waypoints1):
                #reset & reverse the list
                self.waypoints1.reverse()
                self.current_index = 0


    def perform_action(self):
        print("high leveling")
        if not self.current_goal: 
            #for the trajectory
            size = 9.5
            bottom_left = [-size,-size]
            top_right = [size,size]
            diagonal_corners = [bottom_left, top_right]
            num_polygons = 5
            direction = "horizontal"
            self.waypoints = self.get_lawn_mower_pts(diagonal_corners,num_polygons,direction)
            print(self.waypoints)
            #[x,y]
            self.current_index = 0
            self.current_goal = self.waypoints[self.current_index]


            self.waypoints1 = []
            for x,y in self.waypoints:
                self.waypoints1.append(self.create_waypoint(x,y))


        self.publish_waypoints()

    def normalize_quaternion(self,x, y, z, w):
        magnitude = np.sqrt(x**2 + y**2 + z**2 + w**2)
        x /= magnitude
        y /= magnitude
        z /= magnitude
        w /= magnitude
        return x, y, z, w



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

    def quaternion_to_radians(self):
        # Extract the x, y, and z components of the quaternion
        x = self.quaternion.x
        y = self.quaternion.y
        z = self.quaternion.z
        w = self.quaternion.w

        yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))

        return yaw

    def Pythagorean(self,x,y):
        # Calculate the differences in x and y coordinates
        
        # Calculate the square of the differences
        dx_squared = x  ** 2
        dy_squared = y ** 2

        # Calculate the sum of the squares
        sum_of_squares = dx_squared + dy_squared

        # Calculate the square root of the sum of squares
        distance = math.sqrt(sum_of_squares)

        return distance
    

    def custom_waypoint(self,x1,y1):

        print("adjustment")
        action_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        action_client.wait_for_server()

        goal = MoveBaseGoal()
        goal.target_pose = self.create_waypoint(x1,y1)
        action_client.send_goal(goal)
        

        finished_within_time = action_client.wait_for_result(rospy.Duration(900.0))

        # Check the status of the action
        if finished_within_time:
            state = action_client.get_state()
            if state == actionlib.GoalStatus.SUCCEEDED:
                rospy.loginfo('Navigation to the goal succeeded!')
            elif state == actionlib.GoalStatus.PREEMPTED:
                rospy.logwarn('Navigation goal was preempted.')
            elif state == actionlib.GoalStatus.ABORTED:
                rospy.logwarn('Navigation goal aborted.')
            else:
                rospy.logwarn('Navigation goal failed.')
            
            # Get the result of the action
            result = action_client.get_result()
            # Use the result data if needed
            rospy.loginfo('Result: {}'.format(result))
        else:
            # If the action did not complete within the specified timeout
            rospy.logwarn('Navigation did not complete in time.')
        

if __name__ == '__main__':

    node = Waypoint()
    
    while not rospy.is_shutdown():
        node.rate.sleep()
