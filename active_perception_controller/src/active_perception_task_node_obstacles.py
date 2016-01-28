#!/usr/bin/env python

import rospy
import actionlib

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal 
from active_perception_controller.srv import ActivePerceptionPlan
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseWithCovariance
from math import *


_robot_pose = PoseWithCovariance()

def request_plan_client():
    rospy.wait_for_service('plan')   
    try:
	request_plan=rospy.ServiceProxy('plan',ActivePerceptionPlan)
	resp1=request_plan()
	return resp1.path
    except rospy.ServiceException, e:
	print "Service call failed: %s"%e

def robot_pose_cb(msg):
    global _robot_pose
    _robot_pose = msg.pose

if __name__=='__main__':
    rospy.init_node('active_perception_task_node')
    _robot_pose_sub = rospy.Subscriber("amcl_pose",
                                       PoseWithCovarianceStamped,
                                       robot_pose_cb,
                                       queue_size=1)

    simpleac = actionlib.SimpleActionClient("move_base", MoveBaseAction)    
    path=Path()
    simpleac.wait_for_server(rospy.Duration(60))
    while not rospy.is_shutdown():
	index=0
	path=request_plan_client()
	length=len(path.poses)
	while index<length:
		if sqrt((_robot_pose.pose.position.x-path.poses[index].pose.position.x)**2+(_robot_pose.pose.position.y-path.poses[index].pose.position.y)**2)<0.8:
			index=index+1
		goal = MoveBaseGoal()
        	goal.target_pose.header.frame_id = '/map'
        	goal.target_pose.pose.orientation.z = 1
		goal.target_pose.pose.position.x = path.poses[index].pose.position.x
		goal.target_pose.pose.position.y = path.poses[index].pose.position.y
		goal.target_pose.pose.position.z = path.poses[index].pose.position.z
        	simpleac.send_goal(goal)
        	simpleac.wait_for_result()    
    
