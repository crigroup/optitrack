#!/usr/bin/env python
import rospy, os
import numpy as np
# Optitrack
import optirx as rx
from optitrack.utils import get_ip_address, read_parameter
# Transformations
import tf
import tf.transformations as tr
from hrl_geom.pose_converter import PoseConv
# Messages
from geometry_msgs.msg import Pose, Point, Quaternion
from optitrack.msg import RigidBody, RigidBodyArray


class RigidBodiesPublisher(object):
  def __init__(self):
    # Read parameters to configure the calibration detector
    tf_publish_rate = read_parameter('~tf_publish_rate', 50.)
    parent_frame = read_parameter('~parent_frame', 'left/base_link')
    optitrack_frame = read_parameter('~optitrack_frame', 'optitrack')
    rigid_bodies = read_parameter('~rigid_bodies', dict())
    names = []
    ids = []
    for name,value in rigid_bodies.items():
      # TODO: Check required fields
      names.append(name)
      ids.append(value['id'])
    # Setup Publishers
    pose_pub = rospy.Publisher('/optitrack/rigid_bodies', RigidBodyArray, queue_size=3)
    # Setup TF listener and broadcaster
    tf_listener = tf.TransformListener()
    tf_broadcaster = tf.TransformBroadcaster()
    ## Connect to the optitrack system
    optitrack = None
    iface = read_parameter('~iface', 'eth1')
    version = (2, 7, 0, 0)  # the latest SDK version
    optitrack = rx.mkdatasock(ip_address=get_ip_address(iface))
    rospy.loginfo("Successfully connected to optitrack")
    start_time = rospy.get_time()
    prevtime = np.ones(100)*rospy.get_time()    # Track up to 100 rigid bodies
    while not rospy.is_shutdown():
      try:
        data = optitrack.recv(rx.MAX_PACKETSIZE)
      except socket.error:
        if rospy.is_shutdown():
          return    # exit gracefully
        else:
          rospy.logwarn('Failed to receive packet from optitrack')
      packet = rx.unpack(data, version=version)
      if type(packet) is rx.SenderData:
        version = packet.natnet_version
        rospy.loginfo('NatNet version received: ' + str(version))
      if type(packet) in [rx.SenderData, rx.ModelDefs, rx.FrameOfData]:
        rospy.logdebug('Timestamp: %f' % packet.timestamp)
        # Get transformation from optitrack to denso frame
        (parent, child) = (parent_frame, optitrack_frame)
        try:
          (pos,rot) = tf_listener.lookupTransform(parent, child, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException):
          # Wait 2 seconds before showing this warning
          if rospy.get_time() - start_time > 2.0:
            rospy.logwarn('Failed lookupTransform with respect %s frame' % parent)
          continue
        dTo = PoseConv.to_homo_mat(pos, rot)
        # Optitrack gives the position of the centroid. 
        array_msg = RigidBodyArray()
        for i, rigid_body in enumerate(packet.rigid_bodies):
          body_id = rigid_body.id
          pos_opt = np.array(rigid_body.position)
          rot_opt = np.array(rigid_body.orientation)
          oTobj = PoseConv.to_homo_mat(pos_opt, rot_opt)
          # Transformation between parent frame and the rigid body
          dTobj = np.dot(dTo, oTobj)
          array_msg.header.stamp = rospy.Time.now()
          array_msg.header.frame_id = parent_frame
          body_msg = RigidBody()
          pose = Pose()
          pose.position = Point(*dTobj[:3,3])
          pose.orientation = Quaternion(*tr.quaternion_from_matrix(dTobj))
          body_msg.id = body_id
          body_msg.tracking_valid = rigid_body.tracking_valid
          body_msg.mrk_mean_error = rigid_body.mrk_mean_error
          body_msg.pose = pose
          for marker in rigid_body.markers:
            body_msg.markers.append(Point(*marker))
          array_msg.bodies.append(body_msg)
          # TF broadcaster
          if rigid_body.tracking_valid and (rospy.get_time()-prevtime[body_id] >= (1./tf_publish_rate)):
            body_name = 'rigid_body_%d' % (body_id)
            if body_id in ids:
              idx = ids.index(body_id)
              body_name = names[idx]
            tf_broadcaster.sendTransform(pos_opt, rot_opt, rospy.Time.now(), body_name, optitrack_frame)
            prevtime[body_id] = rospy.get_time()
        pose_pub.publish(array_msg)


if __name__ == '__main__':
  node_name = os.path.splitext(os.path.basename(__file__))[0]
  rospy.init_node(node_name)
  rospy.loginfo('Starting [%s] node' % node_name)
  opti_node = RigidBodiesPublisher()
  rospy.loginfo('Shuting down [%s] node' % node_name)
