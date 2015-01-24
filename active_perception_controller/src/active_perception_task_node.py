#!/usr/bin/env python

import rospy
import actionlib

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal    

if __name__=='__main__':
    rospy.init_node('active_perception_task_node')
   
    simpleac = actionlib.SimpleActionClient("move_base", MoveBaseAction)        
    simpleac.wait_for_server(rospy.Duration(60))
    
    while not rospy.is_shutdown():
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = '/map'
        goal.target_pose.pose.orientation.z = 1
        simpleac.send_goal(goal)
        simpleac.wait_for_result()        
