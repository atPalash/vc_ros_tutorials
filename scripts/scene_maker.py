from xml.etree import ElementTree as ET
from stl import mesh
import os

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
      limit = ET.SubElement(self.joint, "limit", lower=limit_l, upper=limit_h, effort=effort_j, velocity=velocity_j)


def scene_stl_generator(stl_src="scene_stl_files/", stl_dst="scene_stl_files_scaled/"):
  source_folder = stl_src
  # robot_name = "abb_irb2400"
  for filename in os.listdir(source_folder):
    # scale the stl files from VC
    obj_stl = mesh.Mesh.from_file(stl_src + filename)
    obj_stl.points = obj_stl.points * 0.001
    obj_stl.save(stl_dst + filename)


if __name__ == "__main__":
  scene_stl_generator()
  #
  # # initialise XACRO creation
  # collision_object = "conveyor1"
  # ET.register_namespace('xacro', "http://ros.org/wiki/xacro")
  #
  # robot = ET.Element(collision_object)
  # macro = ET.SubElement(robot, "{http://ros.org/wiki/xacro}macro", name=collision_object, params="prefix")
  #
  # link_ = Link(macro, collision_object)
  # link_.visual("package://vc_ros/meshes/scene_object/" + collision_object + ".stl")
  # link_.collision("package://vc_ros/meshes/scene_object/" + collision_object + ".stl")
  #
  # Joint(macro, joint_name=collision_object + '-base_link', joint_type='fixed',
  #       origin_xyz='1 0.2 0', origin_rpy='0 0 1.57',
  #       parent_lk="${prefix}" + 'base_link',
  #       child_lk="${prefix}" + collision_object,
  #       )
  #
  # tree = ET.ElementTree(robot)
  # tree.write("xacro/" + robot_name + "/" + robot_name + "_scene_objects" + "_macro.xacro", xml_declaration=True, encoding='utf-8',
  #            method="xml")
