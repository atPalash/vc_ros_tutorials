#!/usr/bin/env python
import json
import os
import threading
import time

import rospy
import moveit_interface as vc_moveit
from geometry_msgs.msg import Pose
from std_msgs.msg import String
from moveit_msgs.msg import PlanningScene, ObjectColor, CollisionObject
from vc_ros_tutorials.srv import vc_ros_connector, vc_ros_connectorResponse

# global variable for moveit and vc connection
moveit_obj = vc_moveit.MoveItDemo()
collision_object_msg = CollisionObject()
dynamic_objects = []
planner_pub = rospy.Publisher("vc_ros_planner", String, queue_size=10)
planner_state = ""


def dynamic_object_pose(data):
    # print data
    global dynamic_objects
    collision_object_pose = Pose()
    data = data.data.split(';')
    if len(data) == 2:
        dynamic_object_name = data[0]
        if "remove" not in data[1]:
            pose_data = json.loads(data[1])
            dynamic_object_pos = pose_data['position']
            dynamic_object_ori = pose_data['orientation']

            collision_object_pose.position.x = dynamic_object_pos['x']
            collision_object_pose.position.y = dynamic_object_pos['y']
            collision_object_pose.position.z = dynamic_object_pos['z']

            collision_object_pose.orientation.x = dynamic_object_ori['x']
            collision_object_pose.orientation.y = dynamic_object_ori['y']
            collision_object_pose.orientation.z = dynamic_object_ori['z']
            collision_object_pose.orientation.w = dynamic_object_ori['w']

            if dynamic_object_name not in dynamic_objects:
                object_p = moveit_obj.generate_pose_msg(vc_moveit.REFERENCE_FRAME, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
                moveit_obj.add_object_to_scene(dynamic_object_name, object_p, dimension=None,
                                               mesh_file="../meshes/scene/Cylinder.stl")

            collision_object_pose.position.z += 0.01
            collision_object_msg.operation = collision_object_msg.MOVE
            collision_object_msg.id = dynamic_object_name
            collision_object_msg.mesh_poses = [collision_object_pose]
            collision_object_msg.header.stamp = rospy.Time.now()
            collision_object_msg.header.frame_id = vc_moveit.REFERENCE_FRAME
        else:
            collision_object_msg.operation = collision_object_msg.REMOVE
            collision_object_msg.id = dynamic_object_name

        moveit_obj.collision_obj_pub.publish(collision_object_msg)


def dynamic_obstruction_pose(data):
    collision_object_pose = Pose()
    data = data.data.split(';')
    if len(data) == 2:
        dynamic_obstruction_name = data[0]

        pose_data = json.loads(data[1])
        dynamic_obs_pos = pose_data['position']
        dynamic_obs_ori = pose_data['orientation']

        collision_object_pose.position.x = dynamic_obs_pos['x']
        collision_object_pose.position.y = dynamic_obs_pos['y']
        collision_object_pose.position.z = dynamic_obs_pos['z']

        collision_object_pose.orientation.x = 0.0
        collision_object_pose.orientation.y = 0.0
        collision_object_pose.orientation.z = 0.0
        collision_object_pose.orientation.w = 1.0

        collision_object_msg.operation = collision_object_msg.MOVE
        collision_object_msg.id = dynamic_obstruction_name
        collision_object_msg.mesh_poses = [collision_object_pose]
        collision_object_msg.header.stamp = rospy.Time.now()
        collision_object_msg.header.frame_id = vc_moveit.REFERENCE_FRAME

        moveit_obj.collision_obj_pub.publish(collision_object_msg)


def planner_state_thread():
    global planner_state, dynamic_objects
    while True:
        planner_pub.publish(planner_state)
        dynamic_objects = moveit_obj.scene.get_objects()
        time.sleep(1)


def service_req_handler(req):
    global planner_state
    msg = req.request_data.data.split(';')
    if len(msg) == 3:
        action = msg[0]
        object_name = msg[1]
        msg = json.loads(msg[2])
        pos = msg['position']
        ori = msg['orientation']
        planner_state = "planning"
        print pos
        if action == "pick":
            grasp_pose = moveit_obj.generate_pose_msg(vc_moveit.REFERENCE_FRAME, [pos['x'], pos['y'], pos['z'] + 0.01],
                                                      [ori['x'], ori['y'], ori['z'], ori['w']])

            grasps = moveit_obj.generate_grasps(grasp_pose, [0.0, 0.0, -1.0], [0.0, 0.0, 1.0], object_name)
            result = moveit_obj.arm_.pick(object_name, grasps, False)
            msg_ = String(data='done')
            planner_state = "done_pick"
            print "pick successful" if result else "pick unsuccessful"
            return vc_ros_connectorResponse(msg_)

        elif action == "place":
            place_pose = moveit_obj.generate_pose_msg(vc_moveit.REFERENCE_FRAME, [pos['x'], pos['y'], pos['z'] + 0.01],
                                                      [ori['x'], ori['y'], ori['z'], ori['w']])
            places = moveit_obj.generate_places(place_pose, [0.0, 0.0, -1.0], [0.0, 0.0, 1.0])
            moveit_obj.arm_.set_support_surface_name("2-Way Conveyor")
            result = moveit_obj.arm_.place(object_name, places, False)
            msg_ = String(data='done')
            planner_state = "done_place"
            print "place successful" if result else "place unsuccessful"
            return vc_ros_connectorResponse(msg_)


if __name__ == "__main__":
    moveit_obj.scene.remove_world_object()
    rospy.sleep(2.0)
    planner = 'EST'
    moveit_obj.arm_.set_planner_id(planner)
    moveit_obj.gripper_.set_planner_id(planner)
    # setting static scene
    scene_stl = "../meshes/scene/"
    for filename in os.listdir(scene_stl):
        if "Cylinder" not in filename:
            filename = filename.split('.')[0]
            object_pose = moveit_obj.generate_pose_msg(vc_moveit.REFERENCE_FRAME, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
            moveit_obj.add_object_to_scene(filename, object_pose, dimension=None,
                                           mesh_file=scene_stl + filename + ".stl")

    # creating subscriber to listen to dynamic object location
    rospy.Subscriber("dynamic_obj_pose", String, dynamic_object_pose)
    rospy.Subscriber("dynamic_obstruction_pose", String, dynamic_obstruction_pose)
    rospy.Service("pose_server", vc_ros_connector, service_req_handler)

    planner_pub_thread = threading.Thread(target=planner_state_thread())
    planner_pub_thread.start()
    rospy.spin()
