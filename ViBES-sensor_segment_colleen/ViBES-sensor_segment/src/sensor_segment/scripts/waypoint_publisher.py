#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped, Point


import numpy as np

def create_waypoint(x, y):
    waypoint = PoseStamped()
    waypoint.header.frame_id = "map"  # Set the frame_id to the correct reference frame
    waypoint.pose.position.x = x
    waypoint.pose.position.y = y
    waypoint.pose.position.z = 0
    
    qx, qy, qz, qw = normalize_quaternion(0, 0, 0, 1)
    
    waypoint.pose.orientation.x = qx
    waypoint.pose.orientation.y = qy
    waypoint.pose.orientation.z = qz
    waypoint.pose.orientation.w = qw
    return waypoint

def publish_waypoints():
    print("waypoints")
    rospy.init_node('waypoint_publisher', anonymous=True)
    action_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
    action_client.wait_for_server()

    # Create an array of waypoints (geometry_msgs/PoseStamped)


    bottom_left = [-9,-9]
    top_right = [9,9]
    diagonal_corners = [bottom_left, top_right]
    #print(diagonal_corners)
    num_polygons = 5
    direction = "horizontal"
    waypoints = get_lawn_mower_pts(diagonal_corners,num_polygons,direction)


    #[x,y]
    # Append your waypoints to the 'waypoints' array

    waypoints1 = []
    for x,y in waypoints:
        waypoints1.append(create_waypoint(x,y))
    print(waypoints)


    #waypoints.append(create_waypoint(3.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0))
    #waypoints.append(create_waypoint(1.0, 1.25, 0.0, 0.0, 0.0, 0.0, 1.0))

    for waypoint in waypoints1:
        goal = MoveBaseGoal()
        print(waypoint)
        goal.target_pose = waypoint
        action_client.send_goal(goal)
        # action_client.wait_for_result(rospy.Duration(10,0))
        # #rospy.sleep(rospy.Duration(10,0))
        # result = action_client.get_result()
        # print(result)
        

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
        

def normalize_quaternion(x, y, z, w):
    magnitude = np.sqrt(x**2 + y**2 + z**2 + w**2)
    x /= magnitude
    y /= magnitude
    z /= magnitude
    w /= magnitude
    return x, y, z, w



def get_lawn_mower_pts(diagonal_corners, num_polygons, direction):
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



if __name__ == '__main__':
    try:
        publish_waypoints()
    except rospy.ROSInterruptException:
        pass

