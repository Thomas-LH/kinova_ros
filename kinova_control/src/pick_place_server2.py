#! /usr/bin/env python

import actionlib
import rospy
import sys
import moveit_commander
import moveit_msgs.msg
import math
import kinova_msgs.msg
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from moveit_msgs.msg import (Constraints, Grasp, GripperTranslation,
                             MoveItErrorCodes, ObjectColor,
                             OrientationConstraint, PlaceLocation,
                             PlanningScene)
from copy import deepcopy
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from actionlib_msgs.msg import GoalStatus
from enum import Enum

GROUP_NAME_ARM = 'arm'
GROUP_NAME_GRIPPER = 'gripper'
GRIPPER_CLOSED = [6400.0, 6400.0, 6400.0]
GRIPPER_OPEN = [0.0, 0.0, 0.0]
GRIPPER_NEUTRAL = [0.7, 0.7, 0.7]
GRIPPER_JOINT_NAMES = ['j2s7s300_joint_finger_1',
                       'j2s7s300_joint_finger_2', 'j2s7s300_joint_finger_3']
GRIPPER_EFFORT = [1.0, 1.0, 1.0]
GRIPPER_FRAME = 'j2s7s300_end_effector'
REFERENCE_FRAME = 'world'

# state definition


class STATE(Enum):
    STOP = 0
    START = 1
    PRE_GRASP_ERR = 2
    GRASP_RETREAT_ERR = 3
    PRE_PLACE_ERR = 4
    PLACE_RETREAT_ERR = 5
    PICK_FINISH = 6
    PLACE_FINISH = 7


class PickPlaceServer2(object):
    _result = kinova_msgs.msg.PoseAndSizeResult()

    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('PP_server2_node')
        self._scene = moveit_commander.PlanningSceneInterface()
        self._scene_pub = rospy.Publisher(
            'planning_scene', PlanningScene, queue_size=2)
        self.state = STATE.START
        self.colors = dict()
        self.max_pick_attempts = 5
        self.max_place_attempts = 5
        # initialize action server, type is PoseAndSizeAction defined in kinova_msgs/action,
        # callback function is pick_and_place
        self._server = actionlib.SimpleActionServer(
            "PickAndPlace", kinova_msgs.msg.PoseAndSizeAction, execute_cb=self.pick_and_place, auto_start=False)
        self._server.start()  # start action server

        # initialize arm and gripper groups
        self.arm = moveit_commander.MoveGroupCommander('arm')
        self.gripper = moveit_commander.MoveGroupCommander('gripper')
        self.arm.allow_replanning(True)
        self.arm.set_planning_time(10)
        end_effector_link = self.arm.get_end_effector_link(
        )  # get end effector link name

        rospy.sleep(2)

        table_back_id = 'table_back'
        tb_side_id = 'back_table_side'
        protectzone_vid = 'pzv'
        protectzone_fid = 'pzf'
        protectzone_sid = 'pzs'
        ground_id = 'ground'
        board_behind_arm_id = 'bba'

        x_offset = 0.28
        table_back_size = [0.5, 0.25, 0.02]  # table on the robot
        tb_side_size = [0.5, 0.02, 0.6]
        pzv_size = [0.7, 0.02, 0.5]  # board before user's face
        pzf_size = [0.5, 0.38, 0.35]  # board above user's foot
        pzs_size = [0.02, 0.5, 1.12]  # board on the right side
        bba_size = [0.2, 0.02, tb_side_size[2] + table_back_size[2]]
        ground_size = [3, 3, 0.02]

        roll_offset = Quaternion(*quaternion_from_euler(0.565, 0, 0))
        self.scene_manage(tb_side_id, tb_side_size, [
                          x_offset + 0.1, 0.0, tb_side_size[2] / 2.0])
        self.scene_manage(table_back_id, table_back_size, [
                          x_offset + 0.1, table_back_size[1] / 2.0, tb_side_size[2] + table_back_size[2] / 2.0])
        self.scene_manage(protectzone_vid, pzv_size, [
                          x_offset, 0.20, tb_side_size[2] + table_back_size[2] + pzv_size[2] / 2.0])
        self.scene_manage(
            ground_id, ground_size, [0.0, 0.0, -ground_size[2] / 2.0])
        self.scene_manage(
            protectzone_fid, pzf_size, [x_offset + 0.1, -0.19, 0.175])
        self.scene_manage(
            board_behind_arm_id, bba_size, [0.03, 0.20, bba_size[2] / 2.0])
        self.scene_manage(
            protectzone_sid, pzs_size, [-0.06, 0.46, pzs_size[2] / 2.0])

        rospy.sleep(1)

        self.set_color(table_back_id, 0.9, 0.9, 0.9)
        self.set_color(tb_side_id, 0.9, 0.9, 0.9)
        self.set_color(protectzone_vid, 0.9, 0.9, 0.9)
        self.set_color(ground_id, 0.3, 0.3, 0.3, 1.0)
        self.set_color(protectzone_fid, 0.9, 0.9, 0.9)
        self.set_color(board_behind_arm_id, 0.9, 0.9, 0.9)
        self.set_color(protectzone_sid, 0.9, 0.9, 0.9)

        # self.arm.set_named_target("Home")
        # self.arm.go()
        # rospy.sleep(1)
        
        pose_init = PoseStamped()
        pose_init.header.frame_id = REFERENCE_FRAME
        pose_init.pose.position.x = -0.288221240044
        pose_init.pose.position.y = -0.313094615936
        pose_init.pose.position.z = 0.395906090736
        pose_init.pose.orientation = Quaternion(
            *quaternion_from_euler(1.57, -1.57, 0))
        self.arm.set_pose_target(pose_init)
        self.arm.go()
        rospy.sleep(1)
        self.gripper.set_named_target("Open")
        self.gripper.go()           
        rospy.sleep(0.5)
        rospy.logdebug(self.arm.get_current_pose())
        '''
        pose_init = rospy.wait_for_message(
            '/j2s7s300_driver/out/tool_pose', PoseStamped)
        pose_init.header.frame_id = REFERENCE_FRAME
        pose_init.pose.orientation = Quaternion(
            *quaternion_from_euler(1.57, -1.57, 0))
        pose_init.pose.position.x -= 0.5
        pose_init.pose.position.y -= 0.05  # -0.05 the the path is better
        self.arm.set_pose_target(pose_init)
        self.arm.go()
        rospy.sleep(1)
        pose_init.pose.position.z -= 0.1
        self.arm.set_pose_target(pose_init)
        self.arm.go()
        rospy.sleep(1)
        
        test_pose = rospy.wait_for_message(
            '/j2s7s300_driver/out/tool_pose', PoseStamped)
        test_pose.pose.orientation = Quaternion(
            *quaternion_from_euler(1.57, -1.57, 0))
        # rospy.logwarn("start_pose:\n%s", test_pose)
        '''
        self.start_pose = deepcopy(pose_init)
        self.arm.remember_joint_values('start_pose')

    def pick_and_place(self, goal):
        # setting object id
        table_front_id = 'table_front'
        target_id = goal.object_class
        rospy.sleep(1)
        # setting object pose and add to the world

        z_offset = 0.01
        # target_size = [goal.object_size.x, 0.055, goal.object_size.y] #
        # length width height --> x z y
        target_size = [0.05, 0.05, 0.18]
        target_position = [
            goal.object_pose.point.x, goal.object_pose.point.y, goal.object_pose.point.z + z_offset]
        table_front_size = [
            1.5, 0.6, goal.object_pose.point.z - target_size[2] / 2.0]
        if target_position[1] > 0.55:
            table_offset = 0.15
        elif target_position[1] >= 0.5:
            table_offset = 0.11
        elif target_position[1] < 0.5:
            table_offset = 0.08
        self.scene_manage(table_front_id, table_front_size, [
                          0.0, -(-goal.object_pose.point.y + table_front_size[1] / 2.0 - target_size[1] / 2.0 - table_offset), table_front_size[2] / 2.0])
        self.scene_manage(target_id, target_size, target_position)

        # setting object color
        self.set_color(table_front_id, 0.8, 0.0, 0.0, 1.0)
        self.set_color(target_id, 0.5, 0.4, 1.0)
        self.send_color()

        grasp_pose = PoseStamped()
        grasp_rpy = [1.57, -1.57, 0.0]
        grasp_pose.header.frame_id = REFERENCE_FRAME
        grasp_pose.pose.position = deepcopy(goal.object_pose.point)
        grasp_pose.pose.orientation = Quaternion(
            *quaternion_from_euler(grasp_rpy[0], grasp_rpy[1], grasp_rpy[2]))
        grasp_pose.pose.position.y += 0.01
        rospy.logwarn("grasp_pose:\n%s", grasp_pose)
        rospy.logwarn("object_size:\n%s", target_size)

        # setting placing position
        place_pose = PoseStamped()
        place_pose.header.frame_id = REFERENCE_FRAME
        self.set_pose(
            place_pose, [target_position[0] - 0.15, target_position[1], table_front_size[2] + target_size[2] / 2.0])
        place_pose.pose.orientation = grasp_pose.pose.orientation

        replan_times = 1
        replan_state = True
        while replan_times <= 3 and replan_state:
            rospy.loginfo("Attempts %d of 3 ", replan_times)
            self.pick(target_id, grasp_pose, 0.05, [0.15, [-1.0, 1.0, 0.0]])
            if self.state is STATE.STOP:
                replan_state = False
                rospy.logwarn('Canceled by user!')
                self._server.set_aborted()
                break
            if self.state is STATE.PRE_GRASP_ERR:
                rospy.logwarn("pre_grasp failed")
                self.state = STATE.START
                self.back_to_init_pose()
            if self.state is STATE.GRASP_RETREAT_ERR:
                rospy.logwarn("grasp_retreat failed")
                current_pose = rospy.wait_for_message(
                    '/j2s7s300_driver/out/tool_pose', PoseStamped)
                current_pose.pose.orientation = Quaternion(
                    *quaternion_from_euler(1.57, -1.57, 0))
                current_pose.pose.position.x -= 0.05
                current_pose.pose.position.y += 0.05
                self.arm.set_pose_target(current_pose)
                self.arm.go()
                self.state = STATE.PICK_FINISH
                rospy.sleep(0.5)

            # self.arm.set_support_surface_name(table_front_id)
            if self.state is STATE.PICK_FINISH:
                self.place(
                    target_id, place_pose, 0.05, [0.10, [-1.0, 1.0, 0.0]])
            if self.state is STATE.PRE_PLACE_ERR:
                rospy.logwarn("pre_place failed")
                while replan_state and replan_times <= 3:
                    rospy.loginfo("Attempts %d of 3 ", replan_times)
                    self.state = STATE.START
                    now = rospy.wait_for_message(
                        '/j2s7s300_driver/out/tool_pose', PoseStamped)
                    rospy.logwarn("pre_place_error--current_pose: \n%s", now)
                    now.pose.orientation = Quaternion(
                        *quaternion_from_euler(grasp_rpy[0], grasp_rpy[1], grasp_rpy[2]))
                    now.pose.position.x += replan_times * \
                        0.02 * ((-1)**replan_times)
                    self.arm.set_pose_target(now)
                    self.arm.go()
                    rospy.sleep(0.5)
                    self.place(
                        target_id, place_pose, 0.05, [0.10, [-1.0, 1.0, 0.0]])
                    replan_times += 1
                    if self.state is STATE.PLACE_FINISH or self.state is STATE.PLACE_RETREAT_ERR:
                        replan_state = False
            if self.state is STATE.PLACE_RETREAT_ERR or self.state is STATE.PLACE_FINISH:
                replan_state = False
                temp = rospy.wait_for_message(
                    '/j2s7s300_driver/out/tool_pose', PoseStamped)
                self._result.arm_pose.point = temp.pose.position
                self._result.arm_pose.header = temp.header
                self._server.set_succeeded(self._result)
                self.back_to_init_pose()
            replan_times += 1

        if replan_state:
            rospy.logerr("Can't find a solution after 3 attempts!")
            if self.state is STATE.PRE_PLACE_ERR:
                for final_attempts in range(1, 4):
                    final_pose = deepcopy(place_pose)
                    rospy.logwarn("place_pose:\n%s", place_pose)
                    final_pose.pose.position.x += final_attempts * \
                        0.01 * ((-1)**final_attempts)
                    # final_pose.pose.position.y -=
                    # final_attempts*0.01*((-1)**final_attempts)
                    rospy.logwarn("final_pose: \n%s", final_pose)
                    self.place(
                        target_id, final_pose, 0.005, [0.10, [0.0, 1.0, 0.0]])
                    if self.state is STATE.PLACE_FINISH or self.state is STATE.PLACE_RETREAT_ERR:
                        self.back_to_init_pose()
                        break
                if self.state is STATE.PRE_PLACE_ERR:
                    rospy.logfatal(
                        "Can't go back to initial pose automatically! Please use joystick!")
                    self.gripper.detach_object(target_id)
            self._server.set_aborted()

    def back_to_init_pose(self, state=0):
        rospy.sleep(0.5)
        print "Return to start posture..."
        if state:
            self.gripper.set_named_target("Open")
            self.gripper.go()
        # self.arm.set_named_target('start_pose')
        self.arm.set_pose_target(self.start_pose)
        self.arm.go()

    def scene_manage(self, obj_id, obj_size, obj_pose, obj_ori=None):
        self._scene.remove_world_object(obj_id)
        obj_pos = PoseStamped()
        obj_pos.header.frame_id = REFERENCE_FRAME
        self.set_pose(obj_pos, obj_pose, orientation=obj_ori)
        self._scene.add_box(obj_id, obj_pos, obj_size)
        rospy.sleep(0.1)

    def set_pose(self, pose_stamped_object, position, orientation=None):
        if type(pose_stamped_object) is not PoseStamped:
            raise Exception(
                'Parameter pose_stamped_object must be a PoseStamped object')
        pose_stamped_object.pose.position.x = position[0]
        pose_stamped_object.pose.position.y = position[1]
        pose_stamped_object.pose.position.z = position[2]

        if orientation is None:
            pose_stamped_object.pose.orientation.w = 1.0
        else:
            pose_stamped_object.pose.orientation = orientation

    def set_color(self, name, r, g, b, a=0.9):
        color = ObjectColor()
        color.id = name
        color.color.r = r
        color.color.g = g
        color.color.b = b
        color.color.a = a
        self.colors[name] = color

    def send_color(self):
        p = PlanningScene()
        p.is_diff = True

        for color in self.colors.values():
            p.object_colors.append(color)

        self._scene_pub.publish(p)
        rospy.sleep(0.1)

    def make_gripper_posture(self, joint_positions):
        t = JointTrajectory()
        t.joint_names = GRIPPER_JOINT_NAMES

        tp = JointTrajectoryPoint()
        tp.positions = joint_positions
        tp.effort = GRIPPER_EFFORT
        t.points.append(tp)

        return t

    def pick(self, target_name, grasp_position, pre_grasp_distance, post_grasp_retreat):
        pre_grasp_posture = JointTrajectory()
        grasp_posture = JointTrajectory()

        pre_grasp_posture = self.make_gripper_posture(GRIPPER_OPEN)
        grasp_posture = self.make_gripper_posture(GRIPPER_CLOSED)

        limit = {'dist': 0.02, 'r': 0.1, 'p': 0.1, 'y': 0.1}
        constraints = Constraints()
        oc = OrientationConstraint()
        oc.header.frame_id = REFERENCE_FRAME
        oc.link_name = 'j2s7s300_end_effector'
        oc.absolute_x_axis_tolerance = limit['r']  # radian
        oc.absolute_y_axis_tolerance = limit['p']
        oc.absolute_z_axis_tolerance = limit['y']
        oc.weight = 1.0
        oc.orientation = grasp_position.pose.orientation
        constraints.orientation_constraints.append(deepcopy(oc))

        pre_approach_point = self.get_retreat_point(
            grasp_position.pose, [0.15, [0.0, 1.0, 0.0]])
        (pre_grasp_approach, fraction) = self.get_path(
            pre_approach_point, 0.005, constraints=constraints)
        if fraction >= 0.90:
            self.arm.execute(pre_grasp_approach)
        result = False
        replan_times = 1
        replan_state = True
        while replan_state and replan_times <= 5:
            (pre_grasp_path, fraction) = self.get_path(
                grasp_position.pose, 0.005, constraints=constraints)
            if fraction >= 0.90:
                print "Pre_grasp_approach..."
                self.arm.execute(pre_grasp_path)
                (result, dis) = self.check(grasp_position.pose, limit)
                if not result:
                    rospy.logwarn("set_pose_target!!!!!!!!!!")
                    self.arm.set_pose_target(grasp_position)
                    self.arm.go()
                    rospy.sleep(0.5)
                    if self.check(grasp_position.pose, limit)[0]:
                        replan_state = False
                else:
                    replan_state = False

            if self._server.current_goal.get_goal_status().status == GoalStatus.PREEMPTING:
                self.state = STATE.STOP
                return
            replan_times += 1
        if self.check(grasp_position.pose, limit)[0]:
            result = False
            if self.gripper.attach_object(
                target_name, self.arm.get_end_effector_link(
                ), ["j2s7s300_end_effector", "j2s7s300_link_finger_1",
                    "j2s7s300_link_finger_tip_1", "j2s7s300_link_finger_2", "j2s7s300_link_finger_tip_2", "j2s7s300_link_finger_3",
                    "j2s7s300_link_finger_tip_3", "j2s7s300_joint_finger_1", "j2s7s300_joint_finger_2", "j2s7s300_joint_finger_3"]):
                rospy.logwarn("Attach request sent successfully!")
            else:
                raise Exception("Can't send attach request!")
            print "Grasping..."
            if self.wait_for_state_update(target_name, box_is_known=False, box_is_attached=True, timeout=4):
                rospy.logwarn("attach!!!!!!!!!!!!!!!!")
            rospy.sleep(1)

            if self._server.current_goal.get_goal_status().status == GoalStatus.PREEMPTING:
                self.gripper.detach_object(target_name)
                self.back_to_init_pose()
                self.state = STATE.STOP
                return

            self.gripper.set_named_target("Close2")
            self.gripper.go()
            rospy.sleep(1)

            post_grasp_position = self.get_retreat_point(
                grasp_position.pose, post_grasp_retreat)

            replan_times = 1
            replan_state = True
            while replan_state and replan_times <= 5:
                (retreat_path, fraction) = self.get_path(
                    post_grasp_position, 0.005, constraints=constraints)
                if self._server.current_goal.get_goal_status().status == GoalStatus.PREEMPTING:
                    self.gripper.detach_object(target_name)
                    self.back_to_init_pose(state=1)
                    self.state = STATE.STOP
                    return
                if fraction > 0.85:
                    print "Retreating..."
                    self.arm.execute(retreat_path)
                    (result, dis) = self.check(post_grasp_position, limit)
                    replan_state = False
                replan_times += 1
            if not result:
                self.state = STATE.GRASP_RETREAT_ERR
        else:
            self.state = STATE.PRE_GRASP_ERR  # pre_grasp planning failed

        if result:
            self.state = STATE.PICK_FINISH
        self.arm.clear_path_constraints()

    def place(self, target_name, place_position, pre_place_distance, post_place_retreat):
        limit = {'dist': 0.04, 'r': 0.10, 'p': 0.10, 'y': 0.10}
        constraints = Constraints()
        oc = OrientationConstraint()
        oc.header.frame_id = REFERENCE_FRAME
        oc.link_name = 'j2s7s300_end_effector'
        oc.absolute_x_axis_tolerance = limit['r']  # radian
        oc.absolute_y_axis_tolerance = limit['p']
        oc.absolute_z_axis_tolerance = limit['y']
        oc.weight = 1
        oc.orientation = place_position.pose.orientation
        constraints.orientation_constraints.append(deepcopy(oc))

        result = False
        replan_times = 1
        replan_state = True
        while replan_state and replan_times <= 5:
            (place_path, fraction) = self.get_path(
                place_position.pose, 0.005, constraints=constraints)
            if fraction >= 0.95:
                print "Pre_place_approach..."
                self.arm.execute(place_path)
                (result, dis) = self.check(place_position.pose, limit)
                replan_state = False
            replan_times += 1

        if result:
            result = False
            print "Placing..."
            self.gripper.set_named_target("Open")
            self.gripper.go()
            rospy.sleep(1)

            self.gripper.detach_object(target_name)
            if self.wait_for_state_update(target_name, box_is_attached=False, box_is_known=True):
                rospy.logwarn("detach!!!!!!!!!!!!!")
            rospy.sleep(1)

            post_approach_position = self.get_retreat_point(place_position.pose, [0.15, [0.0, 1.0, 0.0]])
            rospy.logwarn("post_approach_position:\n%s", post_approach_position)
            rospy.logwarn("start_pose:\n%s", self.start_pose.pose)
            rospy.logwarn("place_pose:\n%s", place_position.pose)
            (post_approach_path, fraction) = self.get_path(post_approach_position, 0.005, constraints=constraints)
            if fraction > 0.9:
                self.arm.execute(post_approach_path)

            post_retreat_pose = rospy.wait_for_message('/j2s7s300_driver/out/tool_pose', PoseStamped)
            post_retreat_pose.pose.orientation = deepcopy(place_position.pose.orientation)

            post_place_position = self.get_retreat_point(
                post_retreat_pose.pose, post_place_retreat)
            replan_state = True
            replan_times = 1
            while replan_state and replan_times <= 5:
                (retreat_path, fraction) = self.get_path(
                    self.start_pose.pose, 0.005, constraints=constraints)
                if fraction > 0.9:
                    self.arm.execute(retreat_path)
                    (result, dis) = self.check(self.start_pose.pose, limit)
                    replan_state = False
                replan_times += 1
            if not result:
                self.state = STATE.PLACE_RETREAT_ERR
        else:
            self.state = STATE.PRE_PLACE_ERR

        if self._server.current_goal.get_goal_status().status == GoalStatus.PREEMPTING:
            rospy.logwarn("Can't cancel task after place action start!")

        if result:
            self.state = STATE.PLACE_FINISH

        self.arm.clear_path_constraints()

    def get_retreat_point(self, grasp_pos, post_grasp_retreat):
        retreat_pos = deepcopy(grasp_pos)
        # compute offset on axis
        offset2 = post_grasp_retreat[0] / pow(2.0, 0.5)
        offset3 = post_grasp_retreat[0] / pow(3.0, 0.5)
        if post_grasp_retreat[1][0]:
            if post_grasp_retreat[1][1]:
                if post_grasp_retreat[1][2]:
                    retreat_pos.position.x += post_grasp_retreat[
                        1][0] * offset3
                    retreat_pos.position.y += post_grasp_retreat[
                        1][1] * offset3
                    retreat_pos.position.z += post_grasp_retreat[
                        1][2] * offset3
                else:
                    retreat_pos.position.x += post_grasp_retreat[
                        1][0] * offset2
                    retreat_pos.position.y += post_grasp_retreat[
                        1][1] * offset2
            else:
                retreat_pos.position.x += post_grasp_retreat[
                    1][0] * post_grasp_retreat[0]
        elif post_grasp_retreat[1][1]:
            if post_grasp_retreat[1][2]:
                retreat_pos.position.y += post_grasp_retreat[1][1] * offset2
                retreat_pos.position.z += post_grasp_retreat[1][2] * offset2
            else:
                retreat_pos.position.y += post_grasp_retreat[
                    1][1] * post_grasp_retreat[0]
        elif post_grasp_retreat[1][2]:
            retreat_pos.position.z += post_grasp_retreat[
                1][2] * post_grasp_retreat[0]
        else:
            raise Exception(
                "No retreat information found. Retreat parameters must be set!")
        return retreat_pos

    def check(self, dest_pos, limit):
        position_check = False
        orientation_check = False
        current_pos = Pose()
        current_pose = rospy.wait_for_message(
            '/j2s7s300_driver/out/tool_pose', PoseStamped)
        current_pose.pose.orientation = Quaternion(
            *quaternion_from_euler(1.57, -1.57, 0.0))
        current_pos = deepcopy(current_pose.pose)
        rospy.logwarn("check()--current_pose:\n%s", current_pos)
        rospy.logwarn("check()--dest_pose:\n%s", dest_pos)
        distance = pow(current_pos.position.x - dest_pos.position.x, 2) + pow(
            current_pos.position.y - dest_pos.position.y, 2) + pow(current_pos.position.z - dest_pos.position.z, 2)
        rospy.logwarn("distance: \n%s", math.sqrt(distance))
        if distance < pow(limit['dist'], 2):
            position_check = True
        dest_rpy = euler_from_quaternion(
            [dest_pos.orientation.x, dest_pos.orientation.y, dest_pos.orientation.z, dest_pos.orientation.w])
        current_rpy = euler_from_quaternion(
            [current_pos.orientation.x, current_pos.orientation.y, current_pos.orientation.z, current_pos.orientation.w])
        if abs(current_rpy[0] - dest_rpy[0]) < limit['r'] and abs(current_rpy[1] - dest_rpy[1]) < limit['p'] and abs(current_rpy[2] - dest_rpy[2]) < limit['y']:
            orientation_check = True

        if position_check and orientation_check:
            rospy.logwarn("Check successfully!")
            return (True, math.sqrt(distance))
        else:
            rospy.logerr("Limition is not satisfied!")
            rospy.logerr("position_check:\n%s", position_check)
            rospy.logerr("orientation_check:\n%s", orientation_check)
            return (False, math.sqrt(distance))

    def get_path(self, dest_position, step, constraints=None):
        # compute path by three provided points(start median end)
        current_pose = PoseStamped()
        current_pose = rospy.wait_for_message(
            '/j2s7s300_driver/out/tool_pose', PoseStamped)
        current_pose.pose.orientation = Quaternion(
            *quaternion_from_euler(1.57, -1.57, 0.0))
        waypoints = []
        waypoints.append(current_pose.pose)
        rospy.logwarn("get_path()--current_pose:\n%s", current_pose)
        wpose = Pose()
        wpose = dest_position
        median = Pose()
        median.position.x = (
            wpose.position.x + current_pose.pose.position.x) / 2.0
        median.position.y = (
            wpose.position.y + current_pose.pose.position.y) / 2.0
        median.position.z = (
            wpose.position.z + current_pose.pose.position.z) / 2.0
        median.orientation = deepcopy(wpose.orientation)
        rospy.logwarn("get_path--median: \n%s", median)
        rospy.logwarn("get_path--dest: \n%s", wpose)
        waypoints.append(deepcopy(median))
        waypoints.append(deepcopy(wpose))

        if constraints is None:
            (plan, fraction) = self.arm.compute_cartesian_path(
                waypoints, step, 0.0)
        else:
            (plan, fraction) = self.arm.compute_cartesian_path(
                waypoints, step, 0.0, path_constraints=constraints)

        return (plan, fraction)

    def wait_for_state_update(self, box_name, box_is_known=False, box_is_attached=False, timeout=4):
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the box is in attached objects
            attached_objects = self._scene.get_attached_objects([box_name])
            is_attached = len(attached_objects.keys()) > 0

            # Test if the box is in the scene.
            # Note that attaching the box will remove it from known_objects
            is_known = box_name in self._scene.get_known_object_names()

            # Test if we are in the expected state
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True

            # Sleep so that we give other threads time on the processor
            rospy.sleep(0.1)
            seconds = rospy.get_time()

        # If we exited the while loop without returning then we timed out
        return False

if __name__ == "__main__":
    try:
        PickPlaceServer2()
        rospy.spin()
    except KeyboardInterrupt:
        raise
