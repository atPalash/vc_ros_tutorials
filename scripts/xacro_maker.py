from stl import mesh
import os
from xml.etree import ElementTree as ET
import yaml


class Link:
    def __init__(self, macro_obj, link_name):
        self.link = ET.SubElement(macro_obj, "link", name="${prefix}" + link_name)

    def visual(self, mesh_file):
        visual = ET.SubElement(self.link, "visual")
        origin = ET.SubElement(visual, "origin", xyz="0 0 0", rpy="0 0 0")
        geometry = ET.SubElement(visual, "geometry")
        mesh = ET.SubElement(geometry, "mesh", filename=mesh_file)

    def collision(self, mesh_file):
        collision = ET.SubElement(self.link, "collision")
        # origin = ET.SubElement(collision, "origin", xyz="0 0 0", rpy="0 0 0")
        geometry = ET.SubElement(collision, "geometry")
        mesh = ET.SubElement(geometry, "mesh", filename=mesh_file)


class Joint:
    def __init__(self, macro_obj, joint_name, joint_type, origin_xyz, origin_rpy, parent_lk, child_lk, axis_xyz=None,
                 limit_l=None, limit_h=None, effort_j=None, velocity_j=None):
        self.joint = ET.SubElement(macro_obj, "joint", name="${prefix}" + joint_name, type=joint_type)
        origin = ET.SubElement(self.joint, "origin", xyz=origin_xyz, rpy=origin_rpy)
        parent = ET.SubElement(self.joint, "parent", link=parent_lk)
        child = ET.SubElement(self.joint, "child", link=child_lk)
        if axis_xyz is not None:
            axis = ET.SubElement(self.joint, "axis", xyz=axis_xyz)
            limit = ET.SubElement(self.joint, "limit", lower=limit_l, upper=limit_h, effort=effort_j,
                                  velocity=velocity_j)


def create_xacro(package_name="vc_ros", rb_name="abb_irb2400", src_stl="robot_stl_files_from_vc/abb_irb2400/",
                 dst_stl="robot_stl_files_from_xacro_maker/abb_irb2400/",
                 yaml_loc="robot_stl_files_from_xacro_maker/abb_irb2400/robot_decription.yaml",
                 dst_xacro="xacro/abb_irb2400/"):
    robot_name = rb_name
    source_folder = src_stl
    destination_folder = dst_stl

    # scale the stl files from VC
    for filename in os.listdir(source_folder):
        if ".stl" in filename:
            link_stl = mesh.Mesh.from_file(source_folder + filename)
            link_stl.points = link_stl.points * 0.001
            link_stl.save(destination_folder + filename)

    # initialise XACRO creation
    ET.register_namespace('xacro', "http://ros.org/wiki/xacro")

    robot = ET.Element("robot")
    macro = ET.SubElement(robot, "{http://ros.org/wiki/xacro}macro", name=robot_name, params="prefix")

    # open yaml file to read robot description
    prev_link = ""
    with open(yaml_loc, 'r') as stream:
        robot_desc = yaml.safe_load(stream)

    # go through the stl files and allocate stl for each link
    count = 0

    for robo_link in robot_desc['link_names']:
        link_ = Link(macro, robo_link)
        if count <= 6:
            link_.visual("package://" + package_name + "/meshes/" + robot_name + "/visual/" + robo_link + ".stl")
            link_.collision("package://" + package_name + "/meshes/" + robot_name + "/visual/" + robo_link + ".stl")
        count += 1

    # allocate joint connecting links
    count = 0
    for robo_joint in robot_desc['joint_names']:
        if count <= 5:
            Joint(macro, joint_name=robo_joint, joint_type=robot_desc['joint_type'][count],
                  origin_xyz=robot_desc['origin_xyz'][count], origin_rpy=robot_desc['origin_rpy'][count],
                  parent_lk="${prefix}" + robot_desc['parent'][count],
                  child_lk="${prefix}" + robot_desc['child'][count],
                  axis_xyz=robot_desc['axis_xyz'][count], limit_l=str(robot_desc['limit_lower'][count]),
                  limit_h=str(robot_desc['limit_upper'][count]), effort_j=str(robot_desc['effort'][count]),
                  velocity_j=str(robot_desc['velocity'][count]))
        # else:
        #     Joint(macro, joint_name=robo_joint, joint_type=robot_desc['joint_type'][count],
        #           origin_xyz=robot_desc['origin_xyz'][count], origin_rpy=robot_desc['origin_rpy'][count],
        #           parent_lk="${prefix}" + robot_desc['parent'][count],
        #           child_lk="${prefix}" + robot_desc['child'][count],
        #           )
        count += 1

    tree = ET.ElementTree(robot)
    tree.write(dst_xacro + robot_name + "_macro.xacro", xml_declaration=True, encoding='utf-8', method="xml")


def create_xacro_caller(package_name="vc_ros", rb_name="abb_irb2400", dst_xacro=None):
    ET.register_namespace('xacro', "http://ros.org/wiki/xacro")

    robot = ET.Element("robot", name=rb_name)
    include = ET.SubElement(robot, "{http://ros.org/wiki/xacro}include",
                            filename="$(find " + package_name + ")/xacro/" +
                                     rb_name + "_macro.xacro")
    prefix = ET.SubElement(robot, "{http://ros.org/wiki/xacro}" + rb_name, prefix="")
    tree = ET.ElementTree(robot)
    tree.write(dst_xacro + "/" + rb_name + ".xacro", xml_declaration=True, encoding='utf-8', method="xml")


if __name__ == "__main__":
    create_xacro()
