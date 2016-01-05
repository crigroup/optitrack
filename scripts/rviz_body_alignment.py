#!/usr/bin/env python
import os, rospy, signal
import numpy as np
# Convex Hull
from scipy.spatial import ConvexHull
# Conversions
import criros as cri
# Messages
from optitrack.msg import RigidBodyArray
# Interactive Markers
from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *
# RViz
from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *
import rviz


class RigidBodyAlignment(QWidget):
  def __init__(self):
    # Initial values
    self.selected = []
    self.Talign = np.eye(4)
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
    # Setup Publishers / Subscribers
    # Look the 
    root = self.manager.getRootDisplayGroup()
    self.body_display = None
    for i in range( root.numDisplays() ):
      if root.getDisplayAt(i).getName() == 'RigidBodyDisplay':
        self.body_display = root.getDisplayAt(i)
        break
    if self.body_display is None:
      rospy.logerr('Failed to find the %r' % 'RigidBodyDisplay')
      exit(1)
    self.topic_prop = self.body_display.subProp('Topic')
    self.id_prop = self.body_display.subProp('Body id')
    self.diameter_prop = self.body_display.subProp('Diameter')
    self.hull_prop = self.body_display.subProp('Convex Hull')
    self.show_hull_prop = self.hull_prop.subProp('Show')
    self.color_prop = self.hull_prop.subProp('Color')
    self.plane_prop = self.body_display.subProp('Plane')
    self.selected_prop = self.plane_prop.subProp('Selected')
    self.normal_prop = self.plane_prop.subProp('Normal')
    self.show_plane_prop = self.plane_prop.subProp('Show')
    self.fit_plane_prop = self.plane_prop.subProp('Fit')
    # QT Connections
    self.id_prop.changed.connect(self.id_changed)
    self.topic_prop.changed.connect(self.topic_changed)
    self.diameter_prop.changed.connect(self.parameter_changed)
    self.show_hull_prop.changed.connect(self.parameter_changed)
    self.color_prop.changed.connect(self.parameter_changed)
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
  
  def clean_msg(self):
    self.selected = []
    try:
      del self.bodies
    except:
      pass
  
  def create_convex_hull(self, points):
    # Control
    control =  InteractiveMarkerControl()
    control.always_visible = True
    # Line strip
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
    color = self.color_prop.getValue().split(';')
    self.hull_color = np.array( map(float, color) ) / 255.
    self.show_hull = self.show_hull_prop.getValue()
    self.snapshot()
  
  def id_changed(self):
    self.clean_msg()
    self.bodyid = self.id_prop.getValue()
    self.snapshot()
  
  def on_shutdown(self, signal=None):
    pass
  
  def received_msg(self):
    return hasattr(self, 'bodies')
  
  def snapshot(self):
    if not self.received_msg():
      self.server.clear()
      self.server.applyChanges()
      return
    for body in self.bodies:
      if not body.tracking_valid or body.id != self.bodyid:
        continue
      num_markers = len(body.markers)
      markers = np.zeros((num_markers,3))
      for i,marker in enumerate(body.markers):
        markers[i,:] = cri.conversions.from_point(marker)
      centroid = cri.conversions.from_point(body.pose.position)
      self.points = (markers - centroid)
      aligned = np.array(self.points)
      # Create interactive marker
      int_marker = InteractiveMarker()
      int_marker.header.frame_id = 'optitrack'
      int_marker.scale = 1
      int_marker.name = 'rigid_body_2'
      int_marker.description = 'rigid_body_2'
      for i,position in enumerate(aligned):
        name = 'marker%02d' % (i+1)
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
    self.clean_msg()
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
