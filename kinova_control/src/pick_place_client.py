#! /usr/bin/env python

from __future__ import print_function
import rospy, sys
import roslib
import actionlib
import std_msgs.msg
import kinova_msgs.msg
from geometry_msgs.msg import PoseStamped

##

def init_func(): 
    topic_addr = "start_state"
    rospy.loginfo("init the subscriber to listening to the arm start state...")
    rospy.Subscriber(topic_addr, kinova_msgs.msg.StateAndObject, pp_client)
    rospy.spin()

def pp_client(feedback):
    if feedback.start:
        rospy.loginfo("calling service")
        client.wait_for_server()

        goal = kinova_msgs.msg.PoseAndSizeGoal()
        goal.object_pose = feedback.object_pose
        goal.object_size = feedback.object_size
        client.send_goal(goal)
        if client.wait_for_result(rospy.Duration(120)):
            print("Pick and Place Action: Succeeded")
            rospy.loginfo(client.get_result().arm_pose)
        else:
            rospy.logwarn("Arm Action Timeout!")

if __name__ == '__main__':
    try:
        rospy.init_node("pick_and_place_node")
        client = actionlib.SimpleActionClient("PickAndPlace", kinova_msgs.msg.PoseAndSizeAction)
        rospy.loginfo("init the pick and place client...")
        init_func()
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)