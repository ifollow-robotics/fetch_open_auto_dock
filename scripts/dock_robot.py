#!/usr/bin/env python

# Copyright (c) 2015, Fetch Robotics Inc.
# Author: Michael Ferguson

import rospy
import actionlib
from fetch_auto_dock_msgs.msg import DockAction, DockGoal

if __name__ == "__main__":
    rospy.init_node("dock_script")
    ACTION_NAME = "/dock"
    rospy.loginfo("Connecting to %s..." % ACTION_NAME)
    client = actionlib.SimpleActionClient(ACTION_NAME, DockAction)
    client.wait_for_server()
    rospy.loginfo("Sending dock goal...")
    goal = DockGoal()
    goal.dock_pose.header.frame_id = "cart_body"
    goal.dock_pose.pose.position.x = 1.9
    goal.dock_pose.pose.position.y = 0.0
    goal.dock_pose.pose.orientation.w = 1.0
    goal.dock_pose.pose.orientation.x = 0.0
    goal.dock_pose.pose.orientation.y = 0.0
    goal.dock_pose.pose.orientation.z = 0.0
    client.send_goal(goal)
    rospy.loginfo("Done, press Ctrl-C to exit")
    rospy.spin()
