#! /usr/bin/env python

import rospy, sys
import moveit_commander
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import PlanningScene, ObjectColor, Grasp, PlaceLocation, GripperTranslation, MoveItErrorCodes, Constraints, OrientationConstraint
from geometry_msgs.msg import PoseStamped, Pose, Quaternion
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from copy import deepcopy

REFERENCE_FRAME  = 'world'

class Moveit_test():
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('moveit_test')

        arm = MoveGroupCommander('arm')
        end_effector_link = arm.get_end_effector_link()
        arm.allow_replanning(True)
        arm.set_planning_time(5)

        target_pose = PoseStamped()
        target_pose.header.frame_id = REFERENCE_FRAME
        target_init_orientation = Quaternion()
        target_init_orientation = quaternion_from_euler(0.0, 1.57, 0.0)
        self.setPose(target_pose, [0.5, 0.2, 0.5],list(target_init_orientation))

        current_pose = arm.get_current_pose(end_effector_link)
        self.setPose(current_pose, [current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z], list(target_init_orientation))
        arm.set_pose_target(current_pose)
        arm.go()
        rospy.sleep(2)


        constraints = Constraints()
        orientation_constraint = OrientationConstraint()
        constraints.name = 'gripper constraint'
        orientation_constraint.header = target_pose.header
        orientation_constraint.link_name = end_effector_link
        orientation_constraint.orientation.x = target_init_orientation[0]
        orientation_constraint.orientation.y = target_init_orientation[1]
        orientation_constraint.orientation.z = target_init_orientation[2]
        orientation_constraint.orientation.w = target_init_orientation[3]

        orientation_constraint.absolute_x_axis_tolerance = 0.03
        orientation_constraint.absolute_y_axis_tolerance = 0.03
        orientation_constraint.absolute_z_axis_tolerance = 3.14
        orientation_constraint.weight = 1.0

        constraints.orientation_constraints.append(orientation_constraint)
        arm.set_path_constraints(constraints)

        arm.set_pose_target(target_pose, end_effector_link)
        arm.go()
        rospy.sleep(1)

        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)
        

    def setPose(self, pose_stamped_object, position, orientation=None):
        if type(pose_stamped_object) is not PoseStamped:
            raise Exception('Parameter pose_stamped_object must be a PoseStamped object')
        pose_stamped_object.pose.position.x = position[0]
        pose_stamped_object.pose.position.y = position[1]
        pose_stamped_object.pose.position.z = position[2]

        if orientation is None:
            pose_stamped_object.pose.orientation.w = 1.0
        else:
            pose_stamped_object.pose.orientation.x = orientation[0]
            pose_stamped_object.pose.orientation.y = orientation[1]
            pose_stamped_object.pose.orientation.z = orientation[2]
            pose_stamped_object.pose.orientation.w = orientation[3] 


if __name__ == '__main__':
    try:
        Moveit_test()
    except KeyboardInterrupt:
        raise

    

