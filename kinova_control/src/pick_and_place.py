#! /usr/bin/env python

import rospy, sys
import moveit_commander
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import PlanningScene, ObjectColor, Grasp, PlaceLocation, GripperTranslation, MoveItErrorCodes, Constraints, OrientationConstraint
from geometry_msgs.msg import PoseStamped, Pose, Quaternion
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from copy import deepcopy


GROUP_NAME_ARM = 'arm'
GROUP_NAME_GRIPPER = 'gripper'
GRIPPER_CLOSED = [1.2, 1.2, 1.2]
GRIPPER_OPEN = [0.2, 0.2, 0.2]
GRIPPER_NEUTRAL = [0.7, 0.7, 0.7]
GRIPPER_JOINT_NAMES = ['j2s7s300_joint_finger_1', 'j2s7s300_joint_finger_2', 'j2s7s300_joint_finger_3']
GRIPPER_EFFORT = [1.0, 1.0, 1.0]
GRIPPER_FRAME = 'j2s7s300_end_effector'
REFERENCE_FRAME  = 'world'

class MoveitObstacle():
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('moveit_demo')
        scene = PlanningSceneInterface('world')
        self.scene_pub = rospy.Publisher('planning_scene', PlanningScene)
        self.gripper_pose_pub = rospy.Publisher('gripper_pose', PoseStamped)
        self.colors = dict()
        max_pick_attempts = 10
        max_place_attempts = 10
        rospy.sleep(1)

        arm = MoveGroupCommander('arm')
        end_effector_link = arm.get_end_effector_link()
        arm.set_goal_position_tolerance(0.01)
        arm.set_goal_orientation_tolerance(0.05)
        arm.allow_replanning(True)
        arm.set_planning_time(5)

        table_id = 'table'
        side_id = 'side'
        table2_id = 'table2'
        box1_id = 'box1'
        box2_id = 'box2'
        target_id = 'target'
        wall_id = 'wall'
        ground_id = 'ground'
        scene.remove_world_object(box1_id)
        scene.remove_world_object(box2_id)
        scene.remove_world_object(table_id)
        scene.remove_world_object(table2_id)
        scene.remove_world_object(target_id)
        scene.remove_world_object(wall_id)
        scene.remove_world_object(ground_id)
        scene.remove_world_object(side_id)
        rospy.sleep(1)

        table_ground = 0.55
        table_size = [0.2, 0.7, 0.01]
        table2_size = [0.25, 0.6, 0.01]
        box1_size = [0.02, 0.9, 0.6]
        box2_size = [0.05, 0.05, 0.15]
        target_size = [0.03, 0.06, 0.10]
        wall_size = [0.01, 0.9, 0.6]
        ground_size = [3, 3, 0.01]
        side_size = [0.01, 0.7, table_ground]

        table_pose = PoseStamped()
        table_pose.header.frame_id = REFERENCE_FRAME
        self.setPose(table_pose, [0.7, 0.0, table_ground + table_size[2]/2.0])    
        scene.add_box(table_id, table_pose, table_size)

        side_pose = PoseStamped()
        side_pose.header.frame_id = REFERENCE_FRAME
        self.setPose(side_pose, [0.605, 0.0, side_size[2]/2.0])
        scene.add_box(side_id, side_pose, side_size)

        table2_pose = PoseStamped()
        table2_pose.header.frame_id = REFERENCE_FRAME
        self.setPose(table2_pose, [-0.325, 0.0, box1_size[2] + table2_size[2]/2.0])
    #    scene.add_box(table2_id, table2_pose, table2_size)

        box1_pose = PoseStamped()
        box1_pose.header.frame_id = REFERENCE_FRAME
        self.setPose(box1_pose, [-0.36, 0.0, box1_size[2]/2.0]) 
    #    scene.add_box(box1_id, box1_pose, box1_size)

        box2_pose = PoseStamped()
        box2_pose.header.frame_id = REFERENCE_FRAME
        self.setPose(box2_pose, [0.63, -0.12, table_ground + table_size[2] + box2_size[2]/2.0])
    #    scene.add_box(box2_id, box2_pose, box2_size)

        target_pose = PoseStamped()
        target_pose.header.frame_id = REFERENCE_FRAME
        self.setPose(target_pose, [0.62, 0.0, table_ground + table_size[2] + target_size[2]/2.0])
        scene.add_box(target_id, target_pose, target_size)

        wall_pose = PoseStamped()
        wall_pose.header.frame_id = REFERENCE_FRAME
        self.setPose(wall_pose, [-0.405, 0.0, box1_size[2] + table2_size[2] + wall_size[2]/2.0])
    #    scene.add_box(wall_id, wall_pose, wall_size)
        
        ground_pose = PoseStamped()
        ground_pose.header.frame_id = REFERENCE_FRAME
        self.setPose(ground_pose, [0.0, 0.0, -ground_size[2]/2.0])
    #    scene.add_box(ground_id, ground_pose, ground_size)

        self.setColor(table_id, 0.8, 0.0, 0.0, 1.0)
    #    self.setColor(table2_id, 0.8, 0.0, 0.0)
    #    self.setColor(box1_id, 0.9, 0.9, 0.9)
    #    self.setColor(box2_id, 0.8, 0.4, 1.0)
        self.setColor(target_id, 0.5, 0.4, 1.0)
    #    self.setColor(wall_id, 0.9, 0.9, 0.9)
        self.setColor(ground_id, 0.3, 0.3, 0.3, 1.0)
        self.setColor(side_id, 0.8, 0.0, 0.0, 1.0)
        self.sendColors()
        
        constraints = Constraints()
        constraints.name = 'gripper constraint'
        orientation_constraint = OrientationConstraint()

        arm.set_support_surface_name(table_id)
        
        test_pose = PoseStamped()
        test_pose.header.frame_id = REFERENCE_FRAME
        test_orientation = Quaternion()
        test_orientation = quaternion_from_euler(0.0, 1.57, -3.14)
        self.setPose(test_pose, [-0.15, -0.22, box1_size[2] + table2_size[2] + target_size[2]/2.0], list(test_orientation))

        place_pose = PoseStamped()
        place_pose.header.frame_id = REFERENCE_FRAME
        self.setPose(place_pose, [0.63, -0.3, table_ground + table_size[2] + target_size[2]/2.0])
        
        grasp_pose = target_pose
        grasp_init_orientation = Quaternion()
        grasp_init_orientation = quaternion_from_euler(0.0, 1.57, 0.0)        
        grasp_pose.pose.orientation.x = grasp_init_orientation[0]
        grasp_pose.pose.orientation.y = grasp_init_orientation[1]
        grasp_pose.pose.orientation.z = grasp_init_orientation[2]
        grasp_pose.pose.orientation.w = grasp_init_orientation[3]
        grasp_pose.pose.position.x -= 0.02
        rospy.loginfo('quaterion: '+ str(grasp_pose.pose.orientation))
        
        orientation_constraint.header = place_pose.header
        orientation_constraint.link_name = end_effector_link
        orientation_constraint.orientation.x = grasp_init_orientation[0]
        orientation_constraint.orientation.y = grasp_init_orientation[1]
        orientation_constraint.orientation.z = grasp_init_orientation[2]
        orientation_constraint.orientation.w = grasp_init_orientation[3]
        orientation_constraint.absolute_x_axis_tolerance = 0.05
        orientation_constraint.absolute_y_axis_tolerance = 0.05
        orientation_constraint.absolute_z_axis_tolerance = 3.14
        orientation_constraint.weight = 1.0
        constraints.orientation_constraints.append(orientation_constraint)
        
        grasps = self.make_grasps(grasp_pose, [table_id], [0.05, 0.07, [1.0, 0.0, 0.0]], [0.22, 0.26, [-1.0, 0.0, 0.0]])
    
        result = None
        n_attempts = 0
        while result != MoveItErrorCodes.SUCCESS and n_attempts < max_pick_attempts:
            result = arm.pick(target_id, grasps)
            n_attempts += 1
            rospy.loginfo('Pick attempt:' + str(n_attempts))
            rospy.sleep(0.2)
        
    #    arm.set_path_constraints(constraints)

        if result == MoveItErrorCodes.SUCCESS:
            result = None
            n_attempts = 0
            places = self.make_places(place_pose, [table_id], [0.15, 0.2, [1.0, 0.0, 0.0]], [0.1, 0.12, [-1.0, 0.0, 0.0]])

        while result != MoveItErrorCodes.SUCCESS and n_attempts < max_place_attempts:
            for place in places:
                result = arm.place(target_id, place)
                if result == MoveItErrorCodes.SUCCESS:
                    break

            n_attempts += 1
            rospy.loginfo('Place attempt:' + str(n_attempts))
            rospy.sleep(0.2)
        
        '''
        grasp_pose = place_pose
        grasp_pose.pose.orientation.x = grasp_init_orientation[0]
        grasp_pose.pose.orientation.y = grasp_init_orientation[1]
        grasp_pose.pose.orientation.z = grasp_init_orientation[2]
        grasp_pose.pose.orientation.w = grasp_init_orientation[3]
        grasp_pose.pose.position.x -= 0.02

        grasps = self.make_grasps(grasp_pose, [table_id], [0.05, 0.07, [1.0, 0.0, 0.0]], [0.15, 0.20, [-1.0, 0.0, 0.0]])
        result = None
        n_attempts = 0
        while result != MoveItErrorCodes.SUCCESS and n_attempts < max_pick_attempts:
            result = arm.pick(target_id, grasps)
            n_attempts += 1
            rospy.loginfo('Pick attempt:' + str(n_attempts))
            rospy.sleep(0.2)
        '''
        
        '''
        arm.set_support_surface_name(table2_id)
        
        self.setPose(place_pose, [-0.26, -0.22, box1_size[2] + table2_size[2] + target_size[2]/2.0])
        
        if result == MoveItErrorCodes.SUCCESS:
            result = None
            n_attempts = 0
            places = self.make_places(place_pose, [table2_id], [0.03, 0.06, [-1.0, 0.0, 0.0]], [0.1, 0.15, [1.0, 0.0, 0.0]])
        
        while result != MoveItErrorCodes.SUCCESS and n_attempts < max_place_attempts:
            for place in places:
                result = arm.place(target_id, place)
                if result == MoveItErrorCodes.SUCCESS:
                    break

            n_attempts += 1
            rospy.loginfo('Place attempt:' + str(n_attempts))
            rospy.sleep(0.2)
        '''
        rospy.sleep(2)
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
    
    def setColor(self, name, r, g, b, a=0.9):
        color = ObjectColor()
        color.id = name
        color.color.r = r
        color.color.g = g
        color.color.b = b
        color.color.a = a
        self.colors[name] = color

    def sendColors(self):
        p = PlanningScene()
        p.is_diff = True

        for color in self.colors.values():
            p.object_colors.append(color)

        self.scene_pub.publish(p)

    def make_grasps(self, initial_pose_stamped, allowed_touch_objects, pre, post):
        g = Grasp()
        g.pre_grasp_posture = self.make_gripper_posture(GRIPPER_OPEN)
        g.grasp_posture = self.make_gripper_posture(GRIPPER_CLOSED)
        g.pre_grasp_approach = self.make_gripper_translation(pre[0], pre[1], pre[2])
        g.post_grasp_retreat = self.make_gripper_translation(post[0], post[1], post[2])

        g.grasp_pose = initial_pose_stamped
        roll_vals = [0, 0.1, -0.1, 0.15, -0.15, 0.25, -0.25]
        yaw_vals = [0]
        pitch_vals = [1.57]
        z_vals =[0]

        grasps = []

        for y in yaw_vals:
            for p in pitch_vals:
                for z in z_vals:
                    for r in roll_vals:
                        q = quaternion_from_euler(r, p, y)
                        g.grasp_pose.pose.orientation.x = q[0]
                        g.grasp_pose.pose.orientation.y = q[1]
                        g.grasp_pose.pose.orientation.z = q[2]
                        g.grasp_pose.pose.orientation.w = q[3]

                        g.grasp_pose.pose.position.z = initial_pose_stamped.pose.position.z + z

                        g.id = str(len(grasps))
                        g.allowed_touch_objects = allowed_touch_objects
                        g.max_contact_force = 0
                        g.grasp_quality = 1.0 - abs(p)
                        grasps.append(deepcopy(g))

        return grasps

    def make_gripper_posture(self, joint_positions):
        t = JointTrajectory()
        t.joint_names = GRIPPER_JOINT_NAMES

        tp = JointTrajectoryPoint()
        tp.positions = joint_positions
        tp.effort = GRIPPER_EFFORT
        t.points.append(tp)

        return t

    def make_gripper_translation(self, min_dist, desired, vector):
        g = GripperTranslation()
        g.direction.vector.x = vector[0]
        g.direction.vector.y = vector[1]
        g.direction.vector.z = vector[2]

        g.direction.header.frame_id = REFERENCE_FRAME   
        g.min_distance = min_dist
        g.desired_distance = desired

        return g

    def make_places(self, init_pose, allowed_touch_objects, pre, post, roll_vals=[0], pitch_vals=[0], yaw_vals=[0]):
        place = PlaceLocation()
        place.post_place_posture = self.make_gripper_posture(GRIPPER_OPEN)
        place.pre_place_approach = self.make_gripper_translation(pre[0], pre[1], pre[2])
        place.post_place_retreat = self.make_gripper_translation(post[0], post[1], post[2])

        place.place_pose = init_pose 
        x_vals = [0, 0.005, 0.01, 0.015, -0.005, -0.01, -0.015]
        y_vals = [0, 0.005, 0.01, 0.015, -0.005, -0.01, -0.015]

        places = []

        for yaw in yaw_vals:
            for pitch in pitch_vals:
                for y in y_vals:
                    for x in x_vals:
                        place.place_pose.pose.position.x = init_pose.pose.position.x + x
                        place.place_pose.pose.position.y = init_pose.pose.position.y + y

                        q = quaternion_from_euler(0, pitch, yaw)

                        place.place_pose.pose.orientation.x = q[0]
                        place.place_pose.pose.orientation.y = q[1]
                        place.place_pose.pose.orientation.z = q[2]
                        place.place_pose.pose.orientation.w = q[3]
                        place.id = str(len(places))
                        place.allowed_touch_objects = allowed_touch_objects

                        places.append(deepcopy(place))
        return places
    
if __name__ == '__main__':
    try:
        MoveitObstacle()
    except KeyboardInterrupt:
        raise