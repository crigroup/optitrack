#!/usr/bin/env python
import argparse
import os, rospy, signal
import numpy as np
import yaml
# Convex Hull
from scipy.spatial import ConvexHull
# Conversions
import criros as cri
import tf.transformations as tr
# Messages
from optitrack.msg import RigidBodyArray
from geometry_msgs.msg import Point
# Interactive Markers
from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *
# RViz
from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *
import rviz


class RigidBodyAlignment(QWidget):
  frame_id = 'optitrack'
  grid_cells = 10
  max_num_bodies = 24
  min_fit_markers = 4
  def __init__(self, config, yaml_path):
    # Initial values
    self.selected = []
    self.marker_names = []
    self.body_display = None
    self.grid_color_prop = None
    self.grid_color = np.ones(3)*0.64
    self.translation = np.zeros(3)
    self.rotation = np.zeros(3)
    # Files paths
    rviz_config = config
    self.yaml_path = yaml_path
    # Initialize RViz
    QWidget.__init__(self)
    self.frame = rviz.VisualizationFrame()
    self.frame.setSplashPath('')
    self.frame.initialize()
    # Load configuration RViz file
    reader = rviz.YamlConfigReader()
    config = rviz.Config()
    reader.readFile(config, rviz_config)
    self.frame.load(config)
    self.manager = self.frame.getManager()
    # Create the layout
    layout = QVBoxLayout()
    layout.addWidget(self.frame)
    self.setLayout(layout)
    # Generic configuration
    np.set_printoptions(precision=5, suppress=True)
    self.server = InteractiveMarkerServer('optitrack/markers')
    # Look-up the RigidBodyDisplay and Grid
    root = self.manager.getRootDisplayGroup()
    for i in range( root.numDisplays() ):
      if root.getDisplayAt(i).getName() == 'RigidBodyDisplay':
        self.body_display = root.getDisplayAt(i)
      elif root.getDisplayAt(i).getName() == 'Grid':
        grid_display = root.getDisplayAt(i)
        self.grid_color_prop = grid_display.subProp('Color')
    if self.body_display is None:
      rospy.logerr('Failed to find the %r' % 'RigidBodyDisplay')
      exit(1)
    # Get the required properties
    self.topic_prop = self.body_display.subProp('Topic')
    self.id_prop = self.body_display.subProp('Body id')
    self.diameter_prop = self.body_display.subProp('Diameter')
    self.hull_prop = self.body_display.subProp('Convex Hull')
    self.show_hull_prop = self.hull_prop.subProp('Show')
    self.color_prop = self.hull_prop.subProp('Color')
    self.plane_prop = self.body_display.subProp('Plane')
    self.selected_prop = self.plane_prop.subProp('Selected')
    self.normal_prop = self.plane_prop.subProp('Normal')
    self.point_prop = self.plane_prop.subProp('Point')
    self.plane_size_prop = self.plane_prop.subProp('Size')
    self.show_plane_prop = self.plane_prop.subProp('Show')
    self.fit_plane_prop = self.plane_prop.subProp('Fit')
    self.transform_prop = self.body_display.subProp('Transform')
    self.translation_prop = self.transform_prop.subProp('Translation')
    self.rotation_prop = self.transform_prop.subProp('Rotation')
    # QT Connections
    self.id_prop.changed.connect(self.id_changed)
    self.topic_prop.changed.connect(self.topic_changed)
    self.diameter_prop.changed.connect(self.parameter_changed)
    self.show_hull_prop.changed.connect(self.parameter_changed)
    self.color_prop.changed.connect(self.parameter_changed)
    if self.grid_color_prop is not None:
      self.grid_color_prop.changed.connect(self.parameter_changed)
    self.normal_prop.changed.connect(self.plane_changed)
    self.point_prop.changed.connect(self.plane_changed)
    self.plane_size_prop.changed.connect(self.plane_changed)
    self.show_plane_prop.changed.connect(self.plane_changed)
    self.fit_plane_prop.changed.connect(self.plane_changed)
    self.translation_prop.changed.connect(self.parameter_changed)
    self.rotation_prop.changed.connect(self.parameter_changed)
    self.plane_changed()
    self.id_changed()
    self.parameter_changed()
    self.topic_changed()
    # Shutdown hook
    rospy.on_shutdown(self.on_shutdown)
    # Load the yaml file if already exist
    self.yaml_dict = None
    if os.path.isfile(self.yaml_path):
      self.yaml_dict = yaml.load(file(self.yaml_path, 'r'))
      # TODO: Check dict consistency
    if self.yaml_dict is None:
      self.yaml_dict = dict()
    
  def cb_optitrack(self, msg):
    if not self.received_msg():
      self.bodies = msg.bodies
      self.snapshot()
  
  def cb_rviz_feedback(self, feedback):
    if feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
      name = feedback.control_name
      if name in self.selected:
        idx = self.selected.index(name)
        self.selected.pop(idx)
      else:
        self.selected.append(name)
      self.update_fit_plane_pop()
      self.snapshot()
  
  def clear_bodies_info(self):
    self.selected = []
    self.update_fit_plane_pop()
    self.fit_plane = False
    self.fit_plane_prop.setValue(False)
    try:
      del self.bodies
    except:
      pass
  
  def create_convex_hull(self, points):
    # Control
    control =  InteractiveMarkerControl()
    control.always_visible = True
    # Triangle list
    marker = Marker()
    marker.type = Marker.TRIANGLE_LIST
    hull = ConvexHull(points)
    for simplex in hull.simplices:
      marker.points.append( cri.conversions.to_point(points[simplex[0],:]) )
      marker.points.append( cri.conversions.to_point(points[simplex[1],:]) )
      marker.points.append( cri.conversions.to_point(points[simplex[2],:]) )
    marker.scale.x = 1.0
    marker.scale.y = 1.0
    marker.scale.z = 1.0
    marker.color.r = self.hull_color[0]
    marker.color.g = self.hull_color[1]
    marker.color.b = self.hull_color[2]
    marker.color.a = 0.5
    # Append
    control.markers.append(marker)
    control.interaction_mode = InteractiveMarkerControl.NONE
    return control
  
  def create_plane_control(self, plane_eq):
    # Create the grid in the XY plane
    linspace =  np.linspace(-0.5,0.5,num=self.grid_cells) * self.plane_size
    xx, yy = np.meshgrid(linspace, linspace)
    grid = []
    for i in range(self.grid_cells):
      # Vertical lines
      grid.append( np.array([xx[0,i], yy[0,i], 0]) )
      grid.append( np.array([xx[-1,i], yy[-1,i], 0]) )
      # Horizontal lines
      grid.append( np.array([xx[i,0], yy[i,0], 0]) )
      grid.append( np.array([xx[i,-1], yy[i,-1], 0]) )
    grid = np.array(grid)
    # Project the grid onto the fitting plane
    T = cri.spalg.transformation_between_planes(plane_eq, [0,0,1,0])
    R = T[:3,:3]
    t = T[:3,3]
    aligned_grid = np.dot(R, grid.T).T + t
    # Control
    control =  InteractiveMarkerControl()
    control.always_visible = True
    # Line list
    marker = Marker()
    marker.type = Marker.LINE_LIST
    marker.scale.x = 0.001
    marker.color.r = self.grid_color[0]
    marker.color.g = self.grid_color[1]
    marker.color.b = self.grid_color[2]
    marker.color.a = 0.75
    for point in aligned_grid:
      marker.points.append( cri.conversions.to_point(point) )
    control.markers.append(marker)
    control.interaction_mode = InteractiveMarkerControl.NONE
    return control
  
  def create_sphere_marker(self, position, color):
    marker = Marker()
    marker.type = Marker.SPHERE
    marker.scale.x = self.diameter
    marker.scale.y = self.diameter
    marker.scale.z = self.diameter
    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    marker.color.a = 1.0
    marker.pose.position = cri.conversions.to_point(position)
    return marker
  
  def create_sphere_control(self, position, selected=False, interactive=True):
    control =  InteractiveMarkerControl()
    control.always_visible = True
    if interactive:
      color = [1, 0, 0] if selected else [0.5, 0.5, 0.5]
    else:
      color = [1, 1, 0]
    marker = self.create_sphere_marker(position, color)
    control.markers.append(marker)
    if interactive:
      control.interaction_mode = InteractiveMarkerControl.BUTTON
    else:
      control.interaction_mode = InteractiveMarkerControl.NONE
    return control
  
  def parameter_changed(self):
    self.diameter = float(self.diameter_prop.getValue()) / 1000.
    color = cri.conversions.from_rviz_vector(self.color_prop.getValue())
    self.hull_color = color / 255.
    self.show_hull = self.show_hull_prop.getValue()
    if self.grid_color_prop is not None:
      color = cri.conversions.from_rviz_vector(self.grid_color_prop.getValue())
      self.grid_color = color / 255.
    self.translation = cri.conversions.from_rviz_vector(self.translation_prop.getValue())
    self.rotation = cri.conversions.from_rviz_vector(self.rotation_prop.getValue())
    self.rotation *= np.pi / 180.
    self.snapshot()
  
  def plane_changed(self):
    self.plane_size = float(self.plane_size_prop.getValue())
    self.fit_plane = self.fit_plane_prop.getValue()
    self.show_plane = self.show_plane_prop.getValue()
    self.plane_normal = cri.conversions.from_rviz_vector(self.normal_prop.getValue())
    self.plane_normal = tr.unit_vector(self.plane_normal)
    if not np.isclose(1.,tr.vector_norm(self.plane_normal)):
      self.plane_normal = np.array([0,1,0])
    self.plane_point = cri.conversions.from_rviz_vector(self.point_prop.getValue())
    self.fit_plane_eq = np.zeros(4)
    self.fit_plane_eq[:3] = np.array(self.plane_normal)
    self.fit_plane_eq[3] = -np.dot(self.plane_normal, self.plane_point)
    self.snapshot()
  
  def id_changed(self):
    self.clear_bodies_info()
    self.bodyid = self.id_prop.getValue()
    self.snapshot()
  
  def on_shutdown(self, signal=None):
    pass
  
  def received_msg(self):
    return hasattr(self, 'bodies')
  
  def snapshot(self):
    self.Talign = tr.euler_matrix(*self.rotation, axes='sxyz')
    self.Talign[:3,3] = self.translation
    # Show the fitting target plane in any case
    self.server.erase('markers_plane')
    self.server.erase('fitting_plane')
    self.server.applyChanges()
    if self.show_plane:
      # TODO: Show the plane center
      int_marker = InteractiveMarker()
      int_marker.header.frame_id = self.frame_id
      int_marker.scale = 1
      int_marker.name = 'fitting_plane'
      control = self.create_plane_control(self.fit_plane_eq)
      int_marker.controls.append( control )
      self.server.insert(int_marker)
      self.server.applyChanges()
    # No rigid bodies info? Clear up the interactive markers
    if not self.received_msg():
      for i in range(self.max_num_bodies):
        self.server.erase('rigid_body_%d' % i)
      self.server.applyChanges()
      return
    # Fit the selected markers to the plane
    if self.fit_plane and len(self.selected) >= self.min_fit_markers:
      markers_to_fit = []
      for name in self.selected:
        idx = self.marker_names.index(name)
        markers_to_fit.append(idx)
      markers_to_fit.sort()
      # Markers plane
      int_marker = InteractiveMarker()
      int_marker.header.frame_id = self.frame_id
      int_marker.scale = 1
      int_marker.name = 'markers_plane'
      markers_plane_eq = cri.spalg.fit_plane_optimize(self.points[markers_to_fit])
      control = self.create_plane_control(markers_plane_eq)
      control.markers[0].color.r, control.markers[0].color.g, control.markers[0].color.b = [0,0,1]
      int_marker.controls.append( control )
      self.server.insert(int_marker)
      self.server.applyChanges()
      # Save fit results
      Tfit = cri.spalg.transformation_between_planes(self.fit_plane_eq, markers_plane_eq)
      self.Talign = np.dot(self.Talign, Tfit)
      self.update_yaml_file()
    # Create the rigid body with the spherical markers and the convex hull
    for body in self.bodies:
      if not body.tracking_valid or body.id != self.bodyid:
        continue
      num_markers = len(body.markers)
      markers = np.zeros((num_markers,3))
      for i,marker in enumerate(body.markers):
        markers[i,:] = cri.conversions.from_point(marker)
      centroid = cri.conversions.from_point(body.pose.position)
      self.points = (markers - centroid)
      R = self.Talign[:3,:3]
      t = self.Talign[:3,3]
      aligned = np.dot(R, self.points.T).T + t
      pivot = t
      # Create interactive marker
      int_marker = InteractiveMarker()
      int_marker.header.frame_id = self.frame_id
      int_marker.scale = 1
      int_marker.name = 'rigid_body_%d' % self.bodyid
      self.marker_names = []
      for i,position in enumerate(aligned):
        name = 'marker%02d' % (i+1)
        self.marker_names.append(name)
        selected = name in self.selected
        control = self.create_sphere_control(position, selected=selected)
        control.name = name
        int_marker.controls.append(control)
      # Add rigid body centroid
      control = self.create_sphere_control(pivot, interactive=False)
      int_marker.controls.append(control)
      if self.show_hull:
        hull = self.create_convex_hull(aligned)
        int_marker.controls.append(hull)
      self.server.insert(int_marker, self.cb_rviz_feedback)
      self.server.applyChanges()
  
  def topic_changed(self):
    try:
      self.sub.unregister()
    except:
      pass
    self.clear_bodies_info()
    self.sub = rospy.Subscriber(self.topic_prop.getValue(), RigidBodyArray, self.cb_optitrack)
  
  def update_fit_plane_pop(self):
    num_selected_markers = len(self.selected)
    if num_selected_markers >= self.min_fit_markers:
      self.fit_plane_prop.show()
    else:
      self.fit_plane_prop.hide()
    self.selected_prop.setValue(num_selected_markers)
  
  def update_yaml_file(self):
    scriptname = os.path.basename(__file__)
    header = '# Translations and rotations were autogenerated by the script %s\n' % scriptname
    header += '# EDITING THOSE FIELDS BY HAND IS NOT RECOMMENDED\n\n'
    Toffset = cri.spalg.transform_inv(self.Talign)
    quaternion = tr.quaternion_from_matrix(Toffset)
    translation = Toffset[:3,3]
    bodyname = 'rigid_body_%d' % self.bodyid
    if not self.yaml_dict.has_key(bodyname):
      self.yaml_dict[bodyname] = dict()
    self.yaml_dict[bodyname]['rotation'] = quaternion.flatten().tolist()
    self.yaml_dict[bodyname]['translation'] = translation.flatten().tolist()
    with open(self.yaml_path, "w") as f:
      f.write(header)
      yaml.safe_dump(self.yaml_dict, f)
    rospy.loginfo('Updated initial transformation for [%s]' % bodyname)
    rospy.loginfo('File [%s] has been updated' % self.yaml_path)


if __name__ == '__main__':
  node_name = os.path.splitext(os.path.basename(__file__))[0]
  rospy.init_node(node_name)
  rospy.loginfo('Starting %r node' % node_name)
  # Parse the arguments
  parser = argparse.ArgumentParser(description='This script helps you to determine the initial orientation of Optitrack rigid bodies')
  parser.add_argument('-c','--config', type=str, required=True,
                        help='Path to the RViz configuration file')
  parser.add_argument('-y', '--yaml', type=str, required=True,
                        help='Path to the yaml file where the initial transformations will be stored/updated')
  args = parser.parse_args(rospy.myargv()[1:])
  app = QApplication(rospy.myargv())
  viz = RigidBodyAlignment(args.config, args.yaml)
  viz.show()
  app.exec_()
  rospy.loginfo('Shuting down %r node' % node_name)
