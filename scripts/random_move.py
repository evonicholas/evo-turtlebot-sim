#!/usr/bin/env python

import rospy
import actionlib
import random
from sensor_msgs.msg import LaserScan
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped
from math import cos, sin

def scan_callback(msg):
    global free_areas

    free_areas = []
    
    for i, distance in enumerate(msg.ranges):
        if distance > 0.6:  # Check for clear areas
            angle = msg.angle_min + i * msg.angle_increment
            free_areas.append((distance, angle))

def set_new_goal():
    global free_areas

    if not free_areas:
        return None

    distance, angle = random.choice(free_areas)

    # Convert polar coordinates to Cartesian coordinates (x, y)
    x = distance * 0.5 * cos(angle)
    y = distance * 0.5 * sin(angle)

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "base_link"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.w = 1.0

    return goal

if __name__ == '__main__':
    rospy.init_node("random_walk_rrt_style")
    
    free_areas = []
    
    client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    rospy.Subscriber("scan", LaserScan, scan_callback)

    client.wait_for_server()

    rate = rospy.Rate(0.5)
    
    while not rospy.is_shutdown():
        goal = set_new_goal()

        if goal:
            client.send_goal(goal)
            client.wait_for_result(rospy.Duration.from_sec(5.0))
        
        rate.sleep()

