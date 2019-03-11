#! /usr/bin/env python

import actionlib
import rospy
import sys
import moveit_commander
import moveit_msgs.msg
import kinova_msgs.msg
import iair_msgs.msg
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
import math

GROUP_NAME_ARM = 'arm'
GROUP_NAME_GRIPPER = 'gripper'
GRIPPER_CLOSED = [6400.0, 6400.0, 6400.0]
GRIPPER_OPEN = [0.0, 0.0, 0.0]
GRIPPER_NEUTRAL = [0.7, 0.7, 0.7]
GRIPPER_JOINT_NAMES = ['j2s7s300_joint_finger_1', 'j2s7s300_joint_finger_2', 'j2s7s300_joint_finger_3']
GRIPPER_EFFORT = [1.0, 1.0, 1.0]
GRIPPER_FRAME = 'j2s7s300_end_effector'
REFERENCE_FRAME  = 'root'

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
        self._scene_pub = rospy.Publisher('planning_scene', PlanningScene, queue_size=2)
        self.state = STATE.START
        self.colors = dict()
        self.max_pick_attempts = 5
        self.max_place_attempts = 5
        
        # initialize action server, type is PoseAndSizeAction defined in kinova_msgs/action,
        # callback function is pick_and_place
        self._server = actionlib.SimpleActionServer("PickAndPlace", kinova_msgs.msg.PoseAndSizeAction, execute_cb=self.pick_and_place, auto_start=False)
        self._server.start() # start action server

        # initialize arm and gripper groups
        self.arm = moveit_commander.MoveGroupCommander('arm')
        self.gripper = moveit_commander.MoveGroupCommander('gripper')
        self.arm.allow_replanning(True)
        self.arm.set_planning_time(10)
        end_effector_link = self.arm.get_end_effector_link() # get end effector link name
        self.status = kinova_msgs.msg.HandStatus()

        rospy.sleep(2) #wait for everything be ready

        table_back_id = 'table_back'
        tb_side_id = 'back_table_side'
        protectzone_vid = 'pzv'
        protectzone_fid = 'pzf'
        protectzone_sid = 'pzs'
        ground_id = 'ground'
        board_behind_arm_id = 'bba'
        
        x_offset = 0.28
        table_back_size = [0.5, 0.25, 0.02] #table on the robot
        tb_side_size = [0.5, 0.02, 0.6]
        pzv_size = [0.7, 0.02, 0.5] #board before user's face
        pzf_size = [0.5, 0.38, 0.35] #board above user's foot
        pzs_size = [0.02, 0.5, 1.12] #board on the right side
        bba_size = [0.2, 0.02, tb_side_size[2] + table_back_size[2]]
        ground_size = [3, 3, 0.02]

        roll_offset = Quaternion(*quaternion_from_euler(0.565, 0, 0))
        self.scene_manage(tb_side_id, tb_side_size, [x_offset + 0.1, 0.0, tb_side_size[2]/2.0])
        self.scene_manage(table_back_id, table_back_size, [x_offset + 0.1, table_back_size[1]/2.0, tb_side_size[2] + table_back_size[2]/2.0])
        self.scene_manage(protectzone_vid, pzv_size, [x_offset, 0.20, tb_side_size[2] + table_back_size[2] + pzv_size[2]/2.0])
        self.scene_manage(ground_id, ground_size, [0.0, 0.0, -ground_size[2]/2.0])
        self.scene_manage(protectzone_fid, pzf_size, [x_offset + 0.1, -0.19, 0.175])
        self.scene_manage(board_behind_arm_id, bba_size, [0.03, 0.20, bba_size[2]/2.0])
        self.scene_manage(protectzone_sid, pzs_size, [-0.06, 0.46, pzs_size[2]/2.0])

        rospy.sleep(0.5)

        self.set_color(table_back_id, 0.9, 0.9, 0.9)
        self.set_color(tb_side_id, 0.9, 0.9, 0.9)
        self.set_color(protectzone_vid, 0.9, 0.9, 0.9)
        self.set_color(ground_id, 0.3, 0.3, 0.3, 1.0)
        self.set_color(protectzone_fid, 0.9, 0.9, 0.9)
        self.set_color(board_behind_arm_id, 0.9, 0.9, 0.9)
        self.set_color(protectzone_sid, 0.9, 0.9, 0.9)

        pose_init = PoseStamped()
        pose_init = self.arm.get_current_pose(end_effector_link)  #this function can't work for some reason, so we subscribe to specific topic to get what we want
        pose_init.header.frame_id = REFERENCE_FRAME
        pose_init.pose.orientation = Quaternion(*quaternion_from_euler(1.57, -1.57, 0))
        pose_init.pose.position.x -= 0.5
        pose_init.pose.position.y -= 0.05 # -0.05 the the path is better
        self.arm.set_pose_target(pose_init)
        self.arm.go()
        rospy.sleep(1)
        pose_init.pose.position.z -= 0.06
        self.arm.set_pose_target(pose_init)
        self.arm.go()
        rospy.sleep(1)
        test_pose = self.arm.get_current_pose()
        rospy.logwarn("start_pose:\n%s", test_pose)
        self.start_pose = deepcopy(test_pose)
        self.arm.remember_joint_values('start_pose')

    def pick_and_place(self, goal):
        # setting object id
        table_front_id = 'table_front'
        target_id = goal.object_class
        rospy.sleep(1)

        # setting object pose and add to the world
        z_offset = 0.01
        target_size = [goal.object_size.x, goal.object_size.y, goal.object_size.z]
        target_position = [goal.object_pose.point.x, goal.object_pose.point.y, goal.object_pose.point.z + z_offset]
        table_front_size = [1.5, 0.6, goal.object_pose.point.z - target_size[2]/2.0]
	if target_position[1] > 0.5:
		table_offset = 0.15
	elif target_position[1] <= 0.5:
		table_offset = 0.08
        self.scene_manage(table_front_id, table_front_size, [0.0, -(-goal.object_pose.point.y + table_front_size[1]/2.0 - target_size[1]/2.0 - table_offset), table_front_size[2]/2.0])
        self.scene_manage(target_id, target_size, target_position)

        # setting object color
        self.set_color(table_front_id, 0.8, 0.0, 0.0, 1.0)
        self.set_color(target_id, 0.5, 0.4, 1.0)
        self.send_color()

        grasp_pose = PoseStamped()
        grasp_rpy = [1.57, -1.57, 0.0]
        grasp_pose.header.frame_id = REFERENCE_FRAME
        grasp_pose.pose.position = deepcopy(goal.object_pose.point)
        grasp_pose.pose.orientation = Quaternion(*quaternion_from_euler(grasp_rpy[0], grasp_rpy[1], grasp_rpy[2]))
        grasp_pose.pose.position.y += 0.01
        rospy.logwarn("grasp_pose:\n%s", grasp_pose)

        # setting placing position
        place_pose = PoseStamped()
        place_pose.header.frame_id = REFERENCE_FRAME
        self.set_pose(place_pose, [target_position[0]-0.15, target_position[1], table_front_size[2] + target_size[2]/2.0])
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
                current_pose = self.arm.get_current_pose()
                current_pose.pose.orientation = Quaternion(*quaternion_from_euler(1.57, -1.57, 0))
                current_pose.pose.position.x -= 0.05
                current_pose.pose.position.y += 0.05
                self.arm.set_pose_target(current_pose)
                self.arm.go()
                self.state = STATE.PICK_FINISH
                rospy.sleep(0.5)

            #self.arm.set_support_surface_name(table_front_id)
            if self.state is STATE.PICK_FINISH:
                self.place(target_id, place_pose, 0.05, [0.15, [-1.0, 1.0, 0.0]])
            if self.state is STATE.PRE_PLACE_ERR:
                rospy.logwarn("pre_place failed")
                while replan_state and replan_times <=3:
                    rospy.loginfo("Attempts %d of 3 ", replan_times)
                    self.state = STATE.START
                    now = self.arm.get_current_pose()
                    now.pose.position.x += replan_times*0.04*(-1)**replan_times
                    self.arm.set_pose_target(now)
                    self.arm.go()
                    rospy.sleep(0.5)
                    self.place(target_id, place_pose, 0.05, [0.15, [-1.0, 1.0, 0.0]])
                    replan_times += 1
                    if self.state is STATE.PLACE_FINISH or self.state is STATE.PLACE_RETREAT_ERR:
                        replan_state = False
            if self.state is STATE.PLACE_RETREAT_ERR or self.state is STATE.PLACE_FINISH:
                replan_state = False
                temp = self.arm.get_current_pose()
                self._result.arm_pose.point = temp.pose.position
                self._result.arm_pose.header = temp.header
                self._server.set_succeeded(self._result)
                self.back_to_init_pose()
            replan_times += 1

        if replan_state:
            rospy.logerr("Can't find a solution after 3 attempts!")
            if self.state is STATE.PRE_PLACE_ERR:
                for final_attempts in range(1,4):
                    final_pose = deepcopy(place_pose)
                    final_pose.pose.position.x += final_attempts*0.02*(-1)**final_attempts
                    final_pose.pose.position.y -= final_attempts*0.02*(-1)**final_attempts
                    self.place(target_id, final_pose, 0.01, [0.15, [-1.0, 1.0, 0.0]])
                    if self.state is STATE.PLACE_FINISH or self.state is STATE.PLACE_RETREAT_ERR:
                        self.back_to_init_pose()
                        break
                if self.state is STATE.PRE_PLACE_ERR:
                    rospy.logfatal("Can't go back to initial pose automatically! Please use joystick!")
		    self.gripper.detach_object(target_id)
            self._server.set_aborted()

    def wait_for_hand_action(self, comm):
        while True:
            self.status = rospy.wait_for_message('hand_status', kinova_msgs.msg.HandStatus)
            if comm == 'close' and self.status.hand_status == kinova_msgs.msg.HandStatus.Close:
                break
            if comm == 'open' and self.status.hand_status == kinova_msgs.msg.HandStatus.Open:
                break        

    def back_to_init_pose(self, state=0):
        rospy.sleep(0.5)
        print "Return to start posture..."
        #if state:
            #self.gripper.set_named_target("Open")
            #self.gripper.go()
        rospy.logwarn(self.arm.get_remembered_joint_values())
        #self.arm.set_named_target('start_pose')
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
            raise Exception('Parameter pose_stamped_object must be a PoseStamped object')
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

        limit = {'dist': 0.01, 'r': 0.15, 'p': 0.15, 'y': 0.15}   
        constraints = Constraints()
        oc = OrientationConstraint()
        oc.header.frame_id = REFERENCE_FRAME
        oc.link_name = 'j2s7s300_end_effector'
        oc.absolute_x_axis_tolerance = limit['r'] # radian
        oc.absolute_y_axis_tolerance = limit['p']
        oc.absolute_z_axis_tolerance = limit['y']
        oc.weight = 1.0
        oc.orientation = grasp_position.pose.orientation
        constraints.orientation_constraints.append(deepcopy(oc))
        
        result = False
        replan_times = 1
        replan_state = True
        while replan_state and replan_times <= 5:
            (pre_grasp_path, fraction) = self.get_path(grasp_position.pose, 0.01, constraints=constraints)
            if self._server.current_goal.get_goal_status().status == GoalStatus.PREEMPTING:
                    self.state = STATE.STOP
                    return
            if fraction >= 0.92:
                print "Pre_grasp_approach..."
                self.arm.execute(pre_grasp_path)
                result = self.check(grasp_position.pose, limit)
                replan_state = False
            replan_times += 1
        if result:
            result = False
            if self.gripper.attach_object(target_name, self.arm.get_end_effector_link(), ["j2s7s300_end_effector", "j2s7s300_link_finger_1", 
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

            #self.wait_for_hand_action('close')
            self.gripper.set_named_target('Close')
            self.gripper.go()

            rospy.sleep(1)
            post_grasp_position = self.get_retreat_point(grasp_position.pose, post_grasp_retreat)
            
            print "post_grasp_position: %s\n" % post_grasp_position
            replan_times = 1
            replan_state = True
            while replan_state and replan_times <= 5:
                (retreat_path, fraction) = self.get_path(post_grasp_position, 0.005, constraints=constraints)
                if self._server.current_goal.get_goal_status().status == GoalStatus.PREEMPTING:
                    self.gripper.detach_object(target_name)
                    self.back_to_init_pose(state=1)
                    self.state = STATE.STOP
                    return
                if fraction > 0.8:
                    print "Retreating..."
                    self.arm.execute(retreat_path)
                    result = self.check(post_grasp_position, limit)
                    replan_state = False
                replan_times += 1
            if not result:
                self.state = STATE.GRASP_RETREAT_ERR
        else:
            self.state = STATE.PRE_GRASP_ERR # pre_grasp planning failed
            
        if result:
            self.state = STATE.PICK_FINISH
        self.arm.clear_path_constraints()


    def place(self, target_name, place_position, pre_place_distance, post_place_retreat):
        limit = {'dist': 0.01, 'r': 0.10, 'p': 0.10, 'y': 0.10}
        constraints = Constraints()
        oc = OrientationConstraint()
        oc.header.frame_id = REFERENCE_FRAME
        oc.link_name = 'j2s7s300_end_effector'
        oc.absolute_x_axis_tolerance = limit['r'] # radian
        oc.absolute_y_axis_tolerance = limit['p']
        oc.absolute_z_axis_tolerance = limit['y']
        oc.weight = 0.85
        oc.orientation = place_position.pose.orientation
        constraints.orientation_constraints.append(deepcopy(oc))

        result = False
        replan_times = 1
        replan_state = True
        while replan_state and replan_times <= 5:
            (place_path, fraction) = self.get_path(place_position.pose, 0.01, constraints=constraints)
            if fraction >= 0.95:
                print "Pre_place_approach..."
                self.arm.execute(place_path)
                result = self.check(place_position.pose, limit)
                replan_state = False
            replan_times += 1

        if result:
            result = False
            print "Placing..."
            #self.wait_for_hand_action('open')
            self.gripper.set_named_target("Open")
            self.gripper.go()
            rospy.sleep(1)

            self.gripper.detach_object(target_name)
            if self.wait_for_state_update(target_name, box_is_attached = False, box_is_known=True):
                rospy.logwarn("detach!!!!!!!!!!!!!")
            rospy.sleep(1)

            post_place_position = self.get_retreat_point(place_position.pose, post_place_retreat)
            replan_state = True
            replan_times = 1
            while replan_state and replan_times <= 5:
                (retreat_path, fraction) = self.get_path(post_place_position, 0.01, constraints=constraints)
                if fraction > 0.8:
                    self.arm.execute(retreat_path)
                    result = self.check(post_place_position, limit)
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
                    retreat_pos.position.x += post_grasp_retreat[1][0] * offset3
                    retreat_pos.position.y += post_grasp_retreat[1][1] * offset3
                    retreat_pos.position.z += post_grasp_retreat[1][2] * offset3
                else:
                    retreat_pos.position.x += post_grasp_retreat[1][0] * offset2
                    retreat_pos.position.y += post_grasp_retreat[1][1] * offset2
            else:
                retreat_pos.position.x += post_grasp_retreat[1][0] * post_grasp_retreat[0]
        elif post_grasp_retreat[1][1]:
            if post_grasp_retreat[1][2]:
                retreat_pos.position.y += post_grasp_retreat[1][1] * offset2
                retreat_pos.position.z += post_grasp_retreat[1][2] * offset2
            else:
                retreat_pos.position.y += post_grasp_retreat[1][1] * post_grasp_retreat[0]
        elif post_grasp_retreat[1][2]:
            retreat_pos.position.z += post_grasp_retreat[1][2] * post_grasp_retreat[0]
        else:
            raise Exception("No retreat information found. Retreat parameters must be set!")
        return retreat_pos

    def check(self, dest_pos, limit):
        position_check = False
        orientation_check = False
        current_pos = Pose()
        current_pos = self.arm.get_current_pose().pose
        rospy.logwarn("current_pose: ")
        print current_pos
        rospy.logwarn("dest_pose: ")
        print dest_pos
        if pow(current_pos.position.x - dest_pos.position.x, 2) + pow(current_pos.position.y - dest_pos.position.y, 2) + pow(current_pos.position.z - dest_pos.position.z, 2) < pow(limit['dist'], 2):
            position_check = True
        dest_rpy = euler_from_quaternion([dest_pos.orientation.x, dest_pos.orientation.y, dest_pos.orientation.z, dest_pos.orientation.w])
        current_rpy = euler_from_quaternion([current_pos.orientation.x, current_pos.orientation.y, current_pos.orientation.z, current_pos.orientation.w])
        if abs(current_rpy[0]-dest_rpy[0]) < limit['r'] and abs(current_rpy[1]-dest_rpy[1]) < limit['p'] and abs(current_rpy[2]-dest_rpy[2]) < limit['y']:
            orientation_check = True

        if position_check and orientation_check:
            return True
        else:
            rospy.logerr("Limition is not satisfied!\n")
            rospy.logerr("position_check: ")
            print position_check
            rospy.logerr("orientation_check: ")
            print orientation_check
            return False
    
    def get_path(self, dest_position, step, constraints=None):
        current_pose = PoseStamped()
        current_pose = self.arm.get_current_pose()
        waypoints = []
        waypoints.append(current_pose.pose)
        rospy.logwarn(current_pose)
        wpose = Pose()
        wpose = dest_position
        waypoints.append(deepcopy(wpose))
        
        if constraints is None:
            (plan, fraction) = self.arm.compute_cartesian_path(waypoints, step, 0.0)
        else:
            (plan, fraction) = self.arm.compute_cartesian_path(waypoints, step, 0.0, path_constraints = constraints)
        
        return (plan, fraction)

    def QuaternionNorm(self, Q_raw):
        qx_temp,qy_temp,qz_temp,qw_temp = Q_raw[0:4]
        qnorm = math.sqrt(qx_temp*qx_temp + qy_temp*qy_temp + qz_temp*qz_temp + qw_temp*qw_temp)
        qx_ = qx_temp/qnorm
        qy_ = qy_temp/qnorm
        qz_ = qz_temp/qnorm
        qw_ = qw_temp/qnorm
        Q_normed_ = [qx_, qy_, qz_, qw_]
        return Q_normed_

    def Quaternion2EulerXYZ(self, Q_raw):
        Q_normed = self.QuaternionNorm(Q_raw)
        qx_, qy_, qz_, qw_  = Q_normed[0:4]
                
        tx_ = math.atan2((2 * qw_ * qx_ - 2 * qy_ * qz_), (qw_ * qw_ - qx_ * qx_ - qy_ * qy_ + qz_ * qz_))
        ty_ = math.asin(2 * qw_ * qy_ + 2 * qx_ * qz_)
        tz_ = math.atan2((2 * qw_ * qz_ - 2 * qx_ * qy_), (qw_ * qw_ + qx_ * qx_ - qy_ * qy_ - qz_ * qz_))
        EulerXYZ_ = [tx_,ty_,tz_]
        return EulerXYZ_


    def EulerXYZ2Quaternion(self, EulerXYZ_):
        tx_, ty_, tz_ = EulerXYZ_[0:3]
        sx = math.sin(0.5 * tx_)
        cx = math.cos(0.5 * tx_)
        sy = math.sin(0.5 * ty_)
        cy = math.cos(0.5 * ty_)
        sz = math.sin(0.5 * tz_)
        cz = math.cos(0.5 * tz_)

        qx_ = sx * cy * cz + cx * sy * sz
        qy_ = -sx * cy * sz + cx * sy * cz
        qz_ = sx * sy * cz + cx * cy * sz
        qw_ = -sx * sy * sz + cx * cy * cz

        Q_ = [qx_, qy_, qz_, qw_]
        return Q_
    
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
