#！ /usr/bin/env python

import rospy, sys
import roslib
import actionlib
import kinova_msgs.msg

class HandServer(object):
    def __init__(self):
        rospy.init_node('hand_server')
        self._server = actionlib.SimpleActionServer('HandServer', kinova_msgs.msg.HandAction, execute_cb=self.hand_handle, auto_start=False)

    def hand_handle(self, goal):
        pass
