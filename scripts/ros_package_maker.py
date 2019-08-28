#! venv/Scripts/python.exe

import os
from scripts import xacro_maker
from scripts import scene_maker


def package_generator(rospack_name, rbt_name, rbt_data, sce_data):
    try:
        os.mkdir("../meshes")
        os.mkdir("../meshes/" + rbt_name)
        os.mkdir("../meshes/" + rbt_name + "/visual")
        os.mkdir("../meshes/scene")

        os.mkdir("../xacro")
        os.mkdir("../srv")
    except Exception as e:
        print e.message

    #xacro_maker.create_xacro(package_name=rospack_name, rb_name=robot_name, src_stl=rbt_data,
    #                        dst_stl="../meshes/" + robot_name + "/visual/",
    #                       yaml_loc=rbt_data + "/description.yaml", dst_xacro="../xacro/")

    #xacro_maker.create_xacro_caller(package_name=rospack_name, rb_name=robot_name, dst_xacro="../xacro")

    scene_maker.scene_stl_generator(stl_src=sce_data, stl_dst="../meshes/scene/")

    srv_file = open("../srv/vc_ros_connector.srv", "w+")
    srv_file.write("std_msgs/String request_data" + "\n" + "---" + "\n" + "std_msgs/String response_data")


if __name__ == "__main__":
    package_name = "vc_ros_tutorials"
    robot_name = "abb_irb2400"
    robot_data_from_vc = "../vc_export/robot/"
    scene_data_from_vc = "../vc_export/scene/"
    package_generator(rospack_name=package_name, rbt_name=robot_name, rbt_data=robot_data_from_vc, sce_data=scene_data_from_vc)
