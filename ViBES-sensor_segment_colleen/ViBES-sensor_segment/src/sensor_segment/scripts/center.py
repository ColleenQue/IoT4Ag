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
    waypoints = [[0,0]]

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


if __name__ == '__main__':
    try:
        publish_waypoints()
    except rospy.ROSInterruptException:
        pass

