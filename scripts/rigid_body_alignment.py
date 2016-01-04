#!/usr/bin/env python
import rospy, os
import numpy as np
# Conversions
import criros as cri
# Messages
from optitrack.msg import RigidBodyArray
# Interactive Markers
from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *


def create_marker(position):
  marker = Marker()
  marker.type = Marker.SPHERE
  marker.scale.x = 0.006
  marker.scale.y = 0.006
  marker.scale.z = 0.006
  marker.color.r = 0.5
  marker.color.g = 0.5
  marker.color.b = 0.5
  marker.color.a = 1.0
  marker.pose.position = cri.conversions.to_point(position)
  return marker
  
def create_control(position):
  control =  InteractiveMarkerControl()
  control.always_visible = True
  control.markers.append( create_marker(position) )
  control.interaction_mode = InteractiveMarkerControl.BUTTON
  return control

class RigidBodyAlignment(object):
  def __init__(self):
    np.set_printoptions(precision=5, suppress=True)
    self.server = InteractiveMarkerServer('optitrack_markers')
    # Setup Publishers / Subscribers
    self.sub = rospy.Subscriber('/optitrack/rigid_bodies', RigidBodyArray, self.cb_optitrack)
    self.connected = False
    while not self.connected:
      rospy.sleep(0.1)
      if rospy.is_shutdown(): return
    rospy.loginfo('Successfully connected to optitrack')
  
  def cb_optitrack(self, msg):
    self.connected = True
    for body in msg.bodies:
      if not body.tracking_valid or body.id != 2:
        continue
      num_markers = len(body.markers)
      markers = np.zeros((num_markers,3))
      for i,marker in enumerate(body.markers):
        markers[i,:] = cri.conversions.from_point(marker)
      centroid = np.mean(markers, axis=0)
      relative = (markers - centroid)
      # Create interactive marker
      int_marker = InteractiveMarker()
      int_marker.header.frame_id = 'world'
      int_marker.scale = 1
      int_marker.name = 'rigid_body_2'
      int_marker.description = 'rigid_body_2'
      for i,position in enumerate(relative):
        control = create_control(position)
        control.name = 'marker%02d' % (i+1)
        int_marker.controls.append(control)
      self.server.insert(int_marker, self.cb_rviz_feedback)
      self.server.applyChanges()
      self.sub.unregister()
    
  def cb_rviz_feedback(self, feedback):
    if feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
      print feedback.control_name


if __name__ == '__main__':
  node_name = os.path.splitext(os.path.basename(__file__))[0]
  rospy.init_node(node_name)
  rospy.loginfo('Starting [%s] node' % node_name)
  align_node = RigidBodyAlignment()
  rospy.spin()
  rospy.loginfo('Shuting down [%s] node' % node_name)
