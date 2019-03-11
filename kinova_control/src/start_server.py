#! /usr/bin/env python

import actionlib
import rospy
import sys
import iair_msgs.msg
import kinova_msgs.msg
from tf import TransformListener
from actionlib_msgs.msg import GoalStatus

class StartServer(object):
    def __init__(self):
        rospy.init_node('start_server')
        self._server = actionlib.SimpleActionServer("StartServer", iair_msgs.msg.armAction, execute_cb=self.call_arm_server, auto_start=False)
        self.tf_listener = TransformListener()
        self.pp_client = actionlib.SimpleActionClient("PickAndPlace", kinova_msgs.msg.PoseAndSizeAction)
        self._server.start()
        
    def call_arm_server(self, goal):
        arm_goal = kinova_msgs.msg.PoseAndSizeGoal()
        arm_goal.object_class = goal.target_object.object_class
        goal.target_object.object_point.header.stamp = rospy.Time(0)
        arm_goal.object_pose = self.tf_listener.transformPoint("world", goal.target_object.object_point)
        rospy.logwarn("arm_goal.object_pose:\n%s", arm_goal.object_pose)
        arm_goal.object_size = goal.target_object.object_size.point
        self.pp_client.wait_for_server()
        self.pp_client.send_goal(arm_goal)
        if self.pp_client.wait_for_result():
            state = self.pp_client.get_state()
            if state == GoalStatus.SUCCEEDED:
                print("Pick and Place Action: Succeeded")
                self._server.set_succeeded()
            elif state == GoalStatus.ABORTED:
                rospy.logwarn("Aborted!")
                self._server.set_aborted()

if __name__ == "__main__":
    try:
        StartServer()
        rospy.spin()
    except KeyboardInterrupt:
        raise