#!/usr/bin/env python
import rospy, os
import numpy as np
# Optitrack
import socket
import optirx as rx
from optitrack.utils import get_ip_address, read_parameter
# Transformations
import tf
import tf.transformations as tr
from hrl_geom.pose_converter import PoseConv
# Messages
from geometry_msgs.msg import Pose, Point, Quaternion
from optitrack.msg import RigidBody, RigidBodyArray

def rigid_body_name(num):
  name = 'rigid_body_%d' % num
  return name

class RigidBodiesPublisher(object):
  """
  Naming convention for the transformations:
    - oTr: Transformation between C{optitrack_frame} and the rigid body
  """
  max_num_bodies = 24
  def __init__(self):
    # Read parameters to configure the node
    tf_publish_rate = read_parameter('~tf_publish_rate', 50.)
    tf_period = 1./tf_publish_rate if tf_publish_rate > 0 else float('inf')
    optitrack_frame = read_parameter('~frame_id', 'optitrack')
    rigid_bodies = dict()
    for i in range(self.max_num_bodies):
      name = '~%s' % rigid_body_name(i+1)
      if rospy.has_param(name):
        rigid_bodies[i+1] = rospy.get_param(name, dict())
    # Setup Publishers
    pose_pub = rospy.Publisher('/optitrack/rigid_bodies', RigidBodyArray, queue_size=3)
    # Setup TF listener and broadcaster
    tf_listener = tf.TransformListener()
    tf_broadcaster = tf.TransformBroadcaster()
    # Connect to the optitrack system
    iface = read_parameter('~iface', 'eth1')
    version = (2, 7, 0, 0)  # the latest SDK version
    optitrack = rx.mkdatasock(ip_address=get_ip_address(iface))
    rospy.loginfo('Successfully connected to optitrack')
    # Track up to the max number of the rigid bodies
    prevtime = np.ones(self.max_num_bodies)*rospy.get_time()
    while not rospy.is_shutdown():
      try:
        data = optitrack.recv(rx.MAX_PACKETSIZE)
      except socket.error:
        if rospy.is_shutdown():  # exit gracefully
          return
        else:
          rospy.logwarn('Failed to receive packet from optitrack')
      packet = rx.unpack(data, version=version)
      if type(packet) is rx.SenderData:
        version = packet.natnet_version
        rospy.loginfo('NatNet version received: ' + str(version))
      if type(packet) in [rx.SenderData, rx.ModelDefs, rx.FrameOfData]:
        # Optitrack gives the position of the centroid. 
        array_msg = RigidBodyArray()
        for i, rigid_body in enumerate(packet.rigid_bodies):
          body_id = rigid_body.id
          pos_opt = np.array(rigid_body.position)
          rot_opt = np.array(rigid_body.orientation)
          array_msg.header.stamp = rospy.Time.now()
          array_msg.header.frame_id = optitrack_frame
          body_msg = RigidBody()
          body_msg.id = body_id
          body_msg.tracking_valid = rigid_body.tracking_valid
          body_msg.mrk_mean_error = rigid_body.mrk_mean_error
          body_msg.pose.position = Point(*pos_opt)
          body_msg.pose.orientation = Quaternion(*rot_opt)
          body_name = rigid_body_name(body_id)
          body_msg.offset.position = Point(0,0,0)
          body_msg.offset.orientation = Quaternion(0,0,0,1)
          if rigid_bodies.has_key(body_id):
            if rigid_bodies[body_id].has_key('name'):
              body_name = rigid_bodies[body_id]['name']
            if rigid_bodies[body_id].has_key('translation'):
              offset_pos = rigid_bodies[body_id]['translation']
              body_msg.offset.position = Point(*offset_pos)
            if rigid_bodies[body_id].has_key('rotation'):
              offset_rot = rigid_bodies[body_id]['rotation']
              body_msg.offset.orientation = Quaternion(*offset_rot)
          body_msg.markers = [Point(*m) for m in rigid_body.markers]
          array_msg.bodies.append(body_msg)
          # Control the publish rate for the TF broadcaster
          if rigid_body.tracking_valid and (rospy.get_time()-prevtime[body_id] >= tf_period):
            oTr = PoseConv.to_homo_mat(body_msg.pose)
            Toffset = PoseConv.to_homo_mat(body_msg.offset)
            T = np.dot(oTr, Toffset)
            pos_tf = T[:3,3]
            rot_tf = tr.quaternion_from_matrix(T)
            tf_broadcaster.sendTransform(pos_tf, rot_tf, rospy.Time.now(), body_name, optitrack_frame)
            prevtime[body_id] = rospy.get_time()
        pose_pub.publish(array_msg)


if __name__ == '__main__':
  node_name = os.path.splitext(os.path.basename(__file__))[0]
  rospy.init_node(node_name)
  rospy.loginfo('Starting [%s] node' % node_name)
  opti_node = RigidBodiesPublisher()
  rospy.loginfo('Shuting down [%s] node' % node_name)
