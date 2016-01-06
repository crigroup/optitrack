#!/usr/bin/env python
import os, rospy, signal
import numpy as np
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
  def __init__(self):
    # Initial values
    self.selected = []
    self.marker_names = []
    self.Talign = np.eye(4)
    self.body_display = None
    self.grid_color_prop = None
    self.grid_color = np.ones(3)*0.64
    # Initialize RViz
    QWidget.__init__(self)
    self.frame = rviz.VisualizationFrame()
    self.frame.setSplashPath('')
    self.frame.initialize()
    # Load configuration RViz file
    reader = rviz.YamlConfigReader()
    config = rviz.Config()
    reader.readFile(config, '/home/fsuarez6/catkin_ws/src/optitrack/config/rviz_body_alignment.rviz')
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
    self.plane_changed()
    self.id_changed()
    self.parameter_changed()
    self.topic_changed()
    # Shutdown hook
    rospy.on_shutdown(self.on_shutdown)
  
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
      num_selected_markers = len(self.selected)
      if num_selected_markers >= 3:
        self.fit_plane_prop.show()
      else:
        self.fit_plane_prop.hide()
      self.selected_prop.setValue(num_selected_markers)
      self.snapshot()
  
  def clear_bodies_info(self):
    self.selected = []
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
  
  def create_plane_control(self, normal, point_on_plane):
    # Create the grid in the XY plane
    linspace =  np.linspace(-0.5,0.5,num=self.grid_cells) * self.plane_size
    xx, yy = np.meshgrid(linspace, linspace)
    points = []
    for i in range(self.grid_cells):  # TODO: There should be a better way to do this
      # Vertical lines
      points.append( np.array([xx[0,i], yy[0,i], 0]) )
      points.append( np.array([xx[-1,i], yy[-1,i], 0]) )
      # Horizontal lines
      points.append( np.array([xx[i,0], yy[i,0], 0]) )
      points.append( np.array([xx[i,-1], yy[i,-1], 0]) )
    points = np.array(points)
    # Align the grid with the plane normal
    R = cri.spalg.rotation_matrix_from_axes(0, normal, oldaxis=[0,0,1])
    #~ aligned_points = np.dot(R[:3,:3], points.T).T + point_on_plane
    #~ points += point_on_plane
    aligned_points = np.dot(R[:3,:3], points.T).T + point_on_plane
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
    for point in aligned_points:
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
  
  def create_sphere_control(self, position, selected=False):
    control =  InteractiveMarkerControl()
    control.always_visible = True
    color = [1, 0, 0] if selected else [0.5, 0.5, 0.5]
    marker = self.create_sphere_marker(position, color)
    control.markers.append(marker)
    control.interaction_mode = InteractiveMarkerControl.BUTTON
    return control
  
  def parameter_changed(self):
    self.diameter = float(self.diameter_prop.getValue()) / 1000.
    color = cri.conversions.from_rviz_vector(self.color_prop.getValue())
    self.hull_color = color / 255.
    self.show_hull = self.show_hull_prop.getValue()
    if self.grid_color_prop is not None:
      color = cri.conversions.from_rviz_vector(self.grid_color_prop.getValue())
      self.grid_color = color / 255.
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
    # Show the fitting plane in any case
    self.server.erase('plane')
    self.server.erase('planes')
    self.server.applyChanges()
    if self.show_plane:
      int_marker = InteractiveMarker()
      int_marker.header.frame_id = self.frame_id
      int_marker.scale = 1
      int_marker.name = 'plane'
      control = self.create_plane_control(self.plane_normal, self.plane_point)
      int_marker.controls.append( control )
      self.server.insert(int_marker)
      self.server.applyChanges()
    # No rigid bodies info? Clear up the interactive markers
    if not self.received_msg():
      for i in range(24):
        self.server.erase('rigid_body_%d' % i)
      self.server.applyChanges()
      return
    # Fit the selected markers to the plane
    if self.fit_plane and len(self.selected) >= 3:
      markers_to_fit = []
      for name in self.selected:
        idx = self.marker_names.index(name)
        markers_to_fit.append(idx)
      markers_to_fit.sort()
      # Debug markers
      # LSTSQ
      int_marker = InteractiveMarker()
      int_marker.header.frame_id = self.frame_id
      int_marker.scale = 1
      int_marker.name = 'planes'
      #~ normal = cri.spalg.fit_plane_lstsq(self.points[markers_to_fit])
      
      #~ control = self.create_plane_control(normal, ctr)
      #~ control.markers[0].color.r, control.markers[0].color.g, control.markers[0].color.b = [0,0,1]
      #~ int_marker.controls.append( control )
      #~ # SVD
      #~ normal = cri.spalg.fit_plane_svd(self.points[markers_to_fit])
      #~ control = self.create_plane_control(normal, ctr)
      #~ control.markers[0].color.r, control.markers[0].color.g, control.markers[0].color.b = [0,1,0]
      #~ int_marker.controls.append( control )
      # Solve
      ctr = np.mean(self.points[markers_to_fit], axis=0)
      centered = self.points[markers_to_fit] - ctr
      normal = cri.spalg.fit_plane_solve(centered*1000.)
      control = self.create_plane_control(normal, ctr)
      #~ control.markers[0].color.r, control.markers[0].color.g, control.markers[0].color.b = [1,0,0]
      int_marker.controls.append( control )
      int_marker.controls.append( control )
      self.server.insert(int_marker)
      self.server.applyChanges()
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
      aligned = np.dot(R,self.points.T).T + t
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

if __name__ == '__main__':
  node_name = os.path.splitext(os.path.basename(__file__))[0]
  rospy.init_node(node_name)
  rospy.loginfo('Starting %r node' % node_name)
  app = QApplication(sys.argv)
  viz = RigidBodyAlignment()
  viz.show()
  import IPython
  IPython.embed()
  #~ app.exec_()
  rospy.loginfo('Shuting down %r node' % node_name)
