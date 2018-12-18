#! /usr/bin/env python

import actionlib
import rospy
import sys
import moveit_commander
import moveit_msgs.msg
import kinova_msgs.msg
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from moveit_msgs.msg import (Constraints, Grasp, GripperTranslation,
                             MoveItErrorCodes, ObjectColor,
                             OrientationConstraint, PlaceLocation,
                             PlanningScene)
from copy import deepcopy
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

GROUP_NAME_ARM = 'arm'
GROUP_NAME_GRIPPER = 'gripper'
GRIPPER_CLOSED = [6400.0, 6400.0, 6400.0]
GRIPPER_OPEN = [0.0, 0.0, 0.0]
GRIPPER_NEUTRAL = [0.7, 0.7, 0.7]
GRIPPER_JOINT_NAMES = ['j2s7s300_joint_finger_1', 'j2s7s300_joint_finger_2', 'j2s7s300_joint_finger_3']
GRIPPER_EFFORT = [1.0, 1.0, 1.0]
GRIPPER_FRAME = 'j2s7s300_end_effector'
REFERENCE_FRAME  = 'world'

class PickPlaceServer2(object):
    _result = kinova_msgs.msg.PoseAndSizeResult()
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('PP_server2_node')
        self._scene = moveit_commander.PlanningSceneInterface('world')
        self._scene_pub = rospy.Publisher('planning_scene', PlanningScene)
        self.colors = dict()
        self.max_pick_attempts = 5
        self.max_place_attempts = 5
        # initialize action server, type is PoseAndSizeAction defined in kinova_msgs/action,
        # callback function is pick_and_place
        self._server = actionlib.SimpleActionServer("PickAndPlace", kinova_msgs.msg.PoseAndSizeAction, execute_cb=self.pick_and_place, auto_start=False)
        self._server.start() # start action server

    def pick_and_place(self, goal):
        self.arm = moveit_commander.MoveGroupCommander('arm')
        self.gripper = moveit_commander.MoveGroupCommander('gripper')
        end_effector_link = self.arm.get_end_effector_link() # get end effector link name
        self.arm.allow_replanning(True)
        self.arm.set_planning_time(10)
        # setting object id
        table_id = 'table'
        side_id = 'side'
        table2_id = 'table2'
        table2_ground_id = 'table2_ground'
        box2_id = 'box2'
        target_id = 'target'
        wall_id = 'wall'
        ground_id = 'ground'
        rospy.sleep(1)
        # setting object size
        table_ground = 0.55
        table_size = [0.2, 0.7, 0.01]
        table2_size = [0.25, 0.6, 0.01]
        table2_ground_size = [0.02, 0.9, 0.6]
        wall_size = [0.01, 0.9, 0.6]
        ground_size = [3, 3, 0.01]
        side_size = [0.01, 0.7, table_ground]
        # setting object pose and add to the world
        self.scene_manage(table2_ground_id, table2_ground_size, [-0.36, 0.0, table2_ground_size[2]/2.0])
        self.scene_manage(table_id, table_size, [0.7, 0.0, table_ground + table_size[2]/2.0])
        self.scene_manage(table2_id, table2_size, [-0.325, 0.0, table2_ground_size[2] + table2_size[2]/2.0])
        self.scene_manage(wall_id, wall_size, [-0.405, 0.0, table2_ground_size[2] + table2_size[2] + wall_size[2]/2.0])
        self.scene_manage(ground_id, ground_size, [0.0, 0.0, -ground_size[2]/2.0])
        self.scene_manage(side_id, side_size, [0.605, 0.0, side_size[2]/2.0])

        z_offset = 0.01
        target_size = [goal.object_size.x, goal.object_size.y, goal.object_size.z]
        target_position = [goal.object_pose.pose.position.x, goal.object_pose.pose.position.y, goal.object_pose.pose.position.z + z_offset]
        self.scene_manage(target_id, target_size, target_position)

        # setting object color
        self.set_color(table_id, 0.8, 0.0, 0.0, 1.0)
        self.set_color(table2_id, 0.8, 0.0, 0.0)
        self.set_color(table2_ground_id, 0.9, 0.9, 0.9)
        self.set_color(target_id, 0.5, 0.4, 1.0)
        self.set_color(wall_id, 0.9, 0.9, 0.9)
        self.set_color(ground_id, 0.3, 0.3, 0.3, 1.0)
        self.set_color(side_id, 0.8, 0.0, 0.0, 1.0)
        self.send_color()

        grasp_rpy = [-3.14, -1.57, 0.0]
        grasp_pose = deepcopy(goal.object_pose)
        grasp_pose.pose.orientation = Quaternion(*quaternion_from_euler(grasp_rpy[0], grasp_rpy[1], grasp_rpy[2]))
        grasp_pose.pose.position.x -= 0.02

        # setting placing position
        place_pose = PoseStamped()
        place_pose.header.frame_id = REFERENCE_FRAME
        self.set_pose(place_pose, [0.63, 0.22, table_ground + table_size[2] + target_size[2]/2.0])
        place_pose.pose.orientation = grasp_pose.pose.orientation

        self.arm.set_named_target("Home")
        self.arm.go()
        rospy.sleep(1)

        pose_init = PoseStamped()
        pose_init = self.arm.get_current_pose(end_effector_link)
        pose_init.pose.orientation = Quaternion(*quaternion_from_euler(grasp_rpy[0], grasp_rpy[1], grasp_rpy[2]))
        self.arm.set_pose_target(pose_init)
        self.arm.go()
        rospy.sleep(1)

        result = self.pick(target_id, grasp_pose, 0.05, [0.15, [-1.0, -1.0, 0.0]])
        if result:
            self.arm.set_support_surface_name(table_id)
            result = self.place(target_id, place_pose, 0.05, [0.15, [-1.0, -1.0, 0.0]])
        if result:
            self._result.arm_pose = self.arm.get_current_pose()
            self._server.set_succeeded(self._result)
        
        rospy.sleep(2)
        print "Return to Home posture..."
        self.arm.set_named_target("Home")
        self.arm.go()

        


    def scene_manage(self, obj_id, obj_size, obj_pose):
        self._scene.remove_world_object(obj_id)
        obj_pos = PoseStamped()
        obj_pos.header.frame_id = REFERENCE_FRAME
        self.set_pose(obj_pos, obj_pose)    
        self._scene.add_box(obj_id, obj_pos, obj_size)

    def set_pose(self, pose_stamped_object, position, orientation=None):
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

        limit = {'dist': 0.005, 'r': 0.15, 'p': 0.15, 'y': 0.15}   
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
        
        reslut = False
        replan_times = 1
        replan_flag = True
        while replan_flag and replan_times <= 5:
            (pre_grasp_path, fraction) = self.get_path(grasp_position.pose, 0.01, constraints=constraints)
            if fraction == 1.0:
                print "Pre_grasp_approach..."
                self.arm.execute(pre_grasp_path)
                result = self.check(grasp_position.pose, limit)
                replan_flag = False
            replan_times += 1
        
        if result:
            result = False
            if self.arm.attach_object(target_name, self.arm.get_end_effector_link(), ["j2s7s300_end_effector", "j2s7s300_link_finger_1", 
            "j2s7s300_link_finger_tip_1", "j2s7s300_link_finger_2", "j2s7s300_link_finger_tip_2", "j2s7s300_link_finger_3", 
            "j2s7s300_link_finger_tip_3", "j2s7s300_joint_finger_1", "j2s7s300_joint_finger_2", "j2s7s300_joint_finger_3"]):
                print "Attach request sent successfully!"
            else:
                raise Exception("Can't send attach request!")
            print "Grasping..."
            self.gripper.set_named_target("Close")
            self.gripper.go()
            rospy.sleep(1)
            post_grasp_position = self.get_retreat_point(grasp_position.pose, post_grasp_retreat)
            
            print "post_grasp_position: %s\n" % post_grasp_position
            replan_times = 1
            replan_flag = True
            while replan_flag and replan_times <= 5:
                (retreat_path, fraction) = self.get_path(post_grasp_position, 0.005, constraints=constraints)
                print "fraction: %s\n" % fraction
                if fraction > 0.8:
                    print "Retreating..."
                    self.arm.execute(retreat_path)
                    result = self.check(post_grasp_position, limit)
                    replan_flag = False
                replan_times += 1
            
        if replan_flag:
            raise Exception("Can't find a solution after 5 attempts!")
            
        self.arm.clear_path_constraints()

        return result

    def place(self, target_name, place_position, pre_place_distance, post_place_retreat):
        limit = {'dist': 0.005, 'r': 0.10, 'p': 0.10, 'y': 0.10}
        constraints = Constraints()
        oc = OrientationConstraint()
        oc.header.frame_id = REFERENCE_FRAME
        oc.link_name = 'j2s7s300_end_effector'
        oc.absolute_x_axis_tolerance = limit['r'] # radian
        oc.absolute_y_axis_tolerance = limit['p']
        oc.absolute_z_axis_tolerance = limit['y']
        oc.weight = 1.0
        oc.orientation = place_position.pose.orientation
        constraints.orientation_constraints.append(deepcopy(oc))

        result = False
        replan_times = 1
        replan_flag = True
        while replan_flag and replan_times <= 5:
            (place_path, fraction) = self.get_path(place_position.pose, 0.01, constraints=constraints)
            if fraction == 1.0:
                print "Pre_place_approach..."
                self.arm.execute(place_path)
                result = self.check(place_position.pose, limit)
                replan_flag = False
            replan_times += 1

        if result:
            result = False
            print "Placing..."
            self.gripper.set_named_target("Open")
            self.gripper.go()
            self.arm.detach_object(target_name)
            rospy.sleep(1)

            post_place_position = self.get_retreat_point(place_position.pose, post_place_retreat)
            replan_flag = True
            replan_times = 1
            while replan_flag and replan_times <= 5:
                (retreat_path, fraction) = self.get_path(post_place_position, 0.01, constraints=constraints)
                if fraction > 0.8:
                    self.arm.execute(retreat_path)
                    result = self.check(post_place_position, limit)
                    replan_flag = False
                replan_times += 1
            
        if replan_flag:
            raise Exception("Can't find a solution after 5 attempts!")

        self.arm.clear_path_constraints()

        return result
    
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
        if pow(current_pos.position.x - dest_pos.position.x, 2) + pow(current_pos.position.y - dest_pos.position.y, 2) + pow(current_pos.position.z - dest_pos.position.z, 2) < pow(limit['dist'], 2):
            position_check = True
        dest_rpy = euler_from_quaternion([dest_pos.orientation.x, dest_pos.orientation.y, dest_pos.orientation.z, dest_pos.orientation.w])
        current_rpy = euler_from_quaternion([current_pos.orientation.x, current_pos.orientation.y, current_pos.orientation.z, current_pos.orientation.w])
        if abs(current_rpy[0]-dest_rpy[0]) < limit['r'] and abs(current_rpy[1]-dest_rpy[1]) < limit['p'] and abs(current_rpy[2]-dest_rpy[2]) < limit['y']:
            orientation_check = True

        if position_check and orientation_check:
            return True
        else:
            return False
    
    def get_path(self, dest_position, step, constraints=None):
        print "current pose: %s" % self.arm.get_current_pose().pose
        waypoints = []
        waypoints.append(self.arm.get_current_pose().pose)
        wpose = Pose()
        wpose = dest_position
        waypoints.append(deepcopy(wpose))
        
        if constraints is None:
            (plan, fraction) = self.arm.compute_cartesian_path(waypoints, step, 0.0)
        else:
            (plan, fraction) = self.arm.compute_cartesian_path(waypoints, step, 0.0, path_constraints = constraints)
        
        return (plan, fraction)

    def safety_check(self):
        pass
        
if __name__ == "__main__":
    try:
        PickPlaceServer2()
        rospy.spin()
    except KeyboardInterrupt:
        raise