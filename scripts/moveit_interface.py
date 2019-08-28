#!/usr/bin/env python

import math
import os

import rospy, sys
import moveit_commander
from geometry_msgs.msg import PoseStamped, Pose
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import PlanningScene, ObjectColor, CollisionObject
from moveit_msgs.msg import Grasp, GripperTranslation, PlaceLocation
from shape_msgs.msg import SolidPrimitive, Plane, Mesh, MeshTriangle

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf.transformations import quaternion_from_euler
from copy import deepcopy

GROUP_NAME_ARM = 'irb_arm'
GROUP_NAME_GRIPPER = 'hand'

GRIPPER_FRAME = 'link6'

GRIPPER_OPEN = [0.04]
GRIPPER_CLOSED = [0.0]
GRIPPER_NEUTRAL = [0.009]
GRIPPER_GRASP = [0.017]

GRIPPER_JOINT_NAMES = ['Axis6']

GRIPPER_EFFORT = [1.0]

REFERENCE_FRAME = 'base_link'

M_PI = math.pi


class MoveItDemo:
    def __init__(self):
        # Initialize the move_group API
        moveit_commander.roscpp_initialize(sys.argv)

        rospy.init_node('moveit_demo')

        # Use the planning scene object to add or remove objects
        self.scene = PlanningSceneInterface()

        # Create a scene publisher to push changes to the scene
        self.scene_pub = rospy.Publisher('planning_scene', PlanningScene, queue_size=5)
        self.collision_obj_pub = rospy.Publisher('collision_object', CollisionObject, queue_size=1)

        # Create a publisher for displaying gripper poses
        self.gripper_pose_pub = rospy.Publisher('gripper_pose', PoseStamped, queue_size=5)

        # Create a dictionary to hold object colors
        self.colors = dict()

        # Initialize the move group for the right arm
        self.arm_ = MoveGroupCommander(GROUP_NAME_ARM)
        self.arm_.set_planner_id("EST")

        # Initialize the move group for the right gripper
        self.gripper_ = MoveGroupCommander(GROUP_NAME_GRIPPER)
        self.gripper_.set_planner_id("EST")

        # Allow replanning to increase the odds of a solution
        self.arm_.allow_replanning(True)

        # Set the right arm reference frame
        # arm_.set_pose_reference_frame(REFERENCE_FRAME)

        # Allow 5 seconds per planning attempt
        self.arm_.set_planning_time(3)

        # Set a limit on the number of pick attempts before bailing
        max_pick_attempts = 5

        # Set a limit on the number of place attempts
        max_place_attempts = 5

        # Give the scene a chance to catch up
        rospy.sleep(2)

    def add_object_to_scene(self, obj_name, object_pose_stamped, dimension=None, mesh_file=None):
        if mesh_file is None:
            self.scene.add_box(obj_name, object_pose_stamped, dimension)
        else:
            self.scene.add_mesh(obj_name, object_pose_stamped, mesh_file)

    @staticmethod
    def generate_pose_msg(frame, position, orientation=None):
        pose_ = PoseStamped()
        pose_.header.frame_id = frame
        pose_.pose.position.x = position[0]
        pose_.pose.position.y = position[1]
        pose_.pose.position.z = position[2]

        if orientation is not None:
            if len(orientation) == 3:
                quaternion_ = quaternion_from_euler(orientation[0], orientation[1], orientation[2])
            elif len(orientation) == 4:
                quaternion_ = orientation
            pose_.pose.orientation.x = quaternion_[0]
            pose_.pose.orientation.y = quaternion_[1]
            pose_.pose.orientation.z = quaternion_[2]
            pose_.pose.orientation.w = quaternion_[3]
        return pose_

    @staticmethod
    def robot_get_pose(group):
        print group.get_current_pose()

    @staticmethod
    def robot_go_to_pose(group, target):
        if type(group) is MoveGroupCommander:
            group.set_pose_target(target)
            group.go()
        else:
            print "group should be of MovegroupCommander type"

    @staticmethod
    def gripper_go_to_pose(group, target):
        if type(group) is MoveGroupCommander:
            group.set_joint_value_target(target)
            group.go()
        else:
            print "group should be of MovegroupCommander type"

    def generate_grasps(self, grasp_pose_stamped, approach_direction, retreat_direction, touch_objects):
        grasps_ = []

        # Initialize the grasp object
        g = Grasp()

        # Set the pre-grasp and grasp postures appropriately
        g.pre_grasp_posture = self.make_gripper_posture(GRIPPER_OPEN)
        g.grasp_posture = self.make_gripper_posture(GRIPPER_GRASP)

        # Set the approach and retreat parameters as desired
        g.pre_grasp_approach = self.make_gripper_translation(0.005, 0.01, approach_direction)
        g.post_grasp_retreat = self.make_gripper_translation(0.005, 0.01, retreat_direction)

        # Set the first grasp pose to the input pose
        g.grasp_pose = grasp_pose_stamped

        g.allowed_touch_objects = touch_objects

        # Don't restrict contact force
        g.max_contact_force = 0

        # Degrade grasp quality for increasing pitch angles
        g.grasp_quality = 1.0

        # Append the grasp to the list
        grasps_.append(deepcopy(g))
        print "Generating Poses"
        # print "Generated " + g.id + " poses"
        # Return the list
        return grasps_

    @staticmethod
    def generate_places(place_pose_stamped, approach, retreat):
        p = PlaceLocation()
        p.place_pose = place_pose_stamped

        p.pre_place_approach.direction.header.frame_id = place_pose_stamped.header.frame_id
        p.pre_place_approach.direction.vector.x = approach[0]
        p.pre_place_approach.direction.vector.y = approach[1]
        p.pre_place_approach.direction.vector.z = approach[2]
        p.pre_place_approach.min_distance = 0.005
        p.pre_place_approach.desired_distance = 0.01

        p.post_place_retreat.direction.header.frame_id = place_pose_stamped.header.frame_id
        p.post_place_retreat.direction.vector.x = retreat[0]
        p.post_place_retreat.direction.vector.y = retreat[1]
        p.post_place_retreat.direction.vector.z = retreat[2]
        p.post_place_retreat.min_distance = 0.005
        p.post_place_retreat.desired_distance = 0.01

        return p

    @staticmethod
    def make_gripper_posture(joint_positions):
        # Initialize the joint trajectory for the gripper joints
        t = JointTrajectory()

        # Set the joint names to the gripper joint names
        t.joint_names = GRIPPER_JOINT_NAMES

        # Initialize a joint trajectory point to represent the goal
        tp = JointTrajectoryPoint()

        # Assign the trajectory joint positions to the input positions
        tp.positions = joint_positions

        # Set the gripper effort
        tp.effort = GRIPPER_EFFORT

        tp.time_from_start = rospy.Duration(1.0)

        # Append the goal point to the trajectory points
        t.points.append(tp)

        # Return the joint trajectory
        return t

    # Generate a gripper translation in the direction given by vector
    @staticmethod
    def make_gripper_translation(min_dist, desired, vector):
        # Initialize the gripper translation object
        g = GripperTranslation()

        # Set the direction vector components to the input
        g.direction.vector.x = vector[0]
        g.direction.vector.y = vector[1]
        g.direction.vector.z = vector[2]

        # The vector is relative to the gripper frame
        g.direction.header.frame_id = REFERENCE_FRAME

        # Assign the min and desired distances from the input
        g.min_distance = min_dist
        g.desired_distance = desired

        return g


if __name__ == "__main__":
    moveit_interface = MoveItDemo()
    moveit_interface.scene.remove_world_object()
    # rospy.sleep(2.0)

    '''
    # add object to scene
    scene_stl = "../meshes/scene/"
    for filename in os.listdir(scene_stl):
        filename = filename.split('.')[0]
        if 'Cylinder' in filename:
            object_pose = moveit_interface.generate_pose_msg(REFERENCE_FRAME, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
            moveit_interface.add_object_to_scene(filename, object_pose, dimension=None,
                                                mesh_file=scene_stl + filename + ".stl")
    rospy.sleep(1)

    object_name = 'Cylinder'
    count = 0
    collision_object_msg = CollisionObject()
    while count < 10:
        scene = moveit_interface.scene
        collision_object_pose = scene.get_object_poses([object_name])[object_name]

        collision_object_pose.position.x += 0.1
        collision_object_pose.position.y = 1.0
        collision_object_pose.position.z = 0.8

        collision_object_msg.operation = collision_object_msg.MOVE

        collision_object_msg.id = object_name
        collision_object_msg.mesh_poses = [collision_object_pose]
        collision_object_msg.header.stamp = rospy.Time.now()
        collision_object_msg.header.frame_id = REFERENCE_FRAME

        moveit_interface.collision_obj_pub.publish(collision_object_msg)
        rospy.sleep(0.1)
        count += 1
    '''
    # create a pose stamped msg, send it to the move group object
    robot_pose = moveit_interface.generate_pose_msg(REFERENCE_FRAME, [-1.0, -0.5, 0.6],
                                                    [-0.249922800852, -0.885554954343, 0.363401602123, 0.145843381843])  # [0.0, M_PI / 2, 0.0]
    # moveit_interface.robot_get_pose(moveit_interface.arm_)
    moveit_interface.robot_go_to_pose(moveit_interface.arm_, robot_pose)
    # specify the group and the values of all joints in a list
    # moveit_interface.gripper_go_to_pose(moveit_interface.gripper_, GRIPPER_OPEN)
    '''
    grasp_pose = moveit_interface.generate_pose_msg(REFERENCE_FRAME, [0.945396421209, 0.966726504932, 0.913051798279], [-8.73825843677e-05, 0.999293463736,
    8.42550460517e-06, 0.0375841140353])  #
    grasps = moveit_interface.generate_grasps(grasp_pose, [0.0, 0.0, -1.0], [0.0, 0.0, 1.0], object_name)
    result = moveit_interface.arm_.pick(object_name, grasps, False)
    print "pick successful" if result else "pick unsuccessful"
    '''
    '''
    place_pose = moveit_interface.generate_pose_msg(REFERENCE_FRAME, [-1.0, -0.8, 0.9], [0.0, 0.0, 0.0])  # [0.0, M_PI/2, 0.0]
    result = moveit_interface.arm_.place(object_name, place_pose, False)
    print "place successful" if result else "place unsuccessful"
    '''




