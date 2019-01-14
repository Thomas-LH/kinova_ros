#! /usr/bin/env python
'''
@Description: A node for arm pick and place action client
@Author: Binghui Li
@Date: 2018-11-01 18:08:55
@LastEditTime: 2018-12-28 16:15:42
@LastEditors: Please set LastEditors
'''

import rospy, sys
import roslib
import actionlib
import std_msgs.msg
import kinova_msgs.msg
import iair_msgs.msg
from geometry_msgs.msg import PoseStamped, Point
from actionlib_msgs.msg import GoalStatus


'''
Initialize a node to subscribe start_state topic
@func: init_func
@msg: kinova_msg.msg.StateAndObject
@return: None
'''
def init_func():
    topic_addr = "start_state"
    rospy.loginfo("init the subscriber to listening to the arm start state...")
    rospy.Subscriber(topic_addr, kinova_msgs.msg.StateAndObject, pp_client)
    rospy.spin()

'''
Callback for initializing a client to call service
@func:pp_client
@action: kinova_msgs.msg.PoseAndSizeAction
@param: feedback 
@return: None
'''

def pp_client(feedback):
    '''
    if feedback.status == kinova_msgs.msg.StateAndObject.START:
        rospy.loginfo("calling service")
        client.wait_for_server() # wait for server to start

        goal = kinova_msgs.msg.PoseAndSizeGoal()
        goal.object_pose = feedback.object_pose
        goal.object_size = feedback.object_size
        client.send_goal(goal)
        if client.wait_for_result(): # wait forever
            state = client.get_state()
            if state == GoalStatus.SUCCEEDED:
                print("Pick and Place Action: Succeeded")
                #rospy.loginfo(client.get_result().arm_pose)
            elif state == GoalStatus.ABORTED:
                rospy.logwarn("Aborted!")
                
    if feedback.status == kinova_msgs.msg.StateAndObject.CANCEL:
        client.cancel_all_goals()
        rospy.logwarn("Send cancel request")
    '''
    client.wait_for_server()
    goal = iair_msgs.msg.armGoal()
    goal.target_object.object_id = 1
    goal.target_object.object_class = 'bottle'
    goal.target_object.object_point.header.frame_id = 'other'
    goal.target_object.object_point.point = Point(0.0, -1.05, 0.52)
    goal.target_object.object_size.header.frame_id = 'other'
    goal.target_object.object_size.point = Point(0.02, 0.02, 0.20)
    goal.target_object.class_conf = 0.9
    client.send_goal(goal)
    if client.wait_for_result():
        rospy.logwarn("Result was received")


if __name__ == '__main__':
    try:
        rospy.init_node("pick_and_place_node")
        '''
        client = actionlib.SimpleActionClient("PickAndPlace", kinova_msgs.msg.PoseAndSizeAction)
        rospy.loginfo("init the pick and place client...")
        '''
        client = actionlib.SimpleActionClient("StartServer", iair_msgs.msg.armAction)
        init_func()
    except rospy.ROSInterruptException:
        print "program interrupted before completion"