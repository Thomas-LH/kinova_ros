#! /usr/bin/env python

import rospy, sys
import roslib
import actionlib
import kinova_msgs.msg
import moveit_commander

HAND_CLOSE = 1
HAND_OPEN = 2

class HandServer(object):
    def __init__(self):
        rospy.init_node('hand_server')
        moveit_commander.roscpp_initialize(sys.argv)
        self._server = actionlib.SimpleActionServer('HandServer', kinova_msgs.msg.HandAction, execute_cb=self.hand_handle, auto_start=False)
        self.hand_status_pub = rospy.Publisher('hand_status', kinova_msgs.msg.HandStatus, queue_size=1)
        self.gripper = moveit_commander.MoveGroupCommander('gripper')
        self._status = kinova_msgs.msg.HandStatus()
        self._result = kinova_msgs.msg.HandResult()
        self._server.start()

    def hand_handle(self, goal):
        if goal.select is HAND_CLOSE:
            self.gripper.set_named_target('Close')
            self.gripper.go()
            rospy.sleep(0.5)
            self._status.hand_status = HAND_CLOSE
            self.hand_status_pub.publish(self._status)
            self._result.result = True
            self._server.set_succeeded(self._result)
        elif goal.select is HAND_OPEN:
            self.gripper.set_named_target('Open')
            self.gripper.go()
            rospy.sleep(0.5)
            self._status.hand_status = HAND_OPEN
            self.hand_status_pub.publish(self._status)
            self._result.result = True
            self._server.set_succeeded(self._result)

if __name__ == "__main__":
    try:
        HandServer()
        rospy.spin()
    except KeyboardInterrupt:
        raise
