#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
        
        
def move(x, y):
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.w = 1.0
    
    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()
  

if __name__ == '__main__':
    rospy.init_node("move_sequence")
    
    goals = [(2.0, 0.5), (-2.0, 0.5), (2.0, -0.5), (2.0, 0)]
    
    client = actionlib.SimpleActionClient("move_base", MoveBaseAction)

    client.wait_for_server()

    rate = rospy.Rate(0.5) 
    try:
        for point in goals:
            result = move(point[0], point[1])
            if result:
                rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")

