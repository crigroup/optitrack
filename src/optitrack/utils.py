#! /usr/bin/env python
import rospy
import socket, fcntl, struct

def get_ip_address(ifname):
  """
  Get the ip address from the specified interface.
    
    >>> get_ip_address('eth0')
    '192.168.0.7'
    
  @type ifname: string
  @param ifname: The interface name. Typical names are C{'eth0'}, 
  C{'wlan0'}, etc.
  @rtype: string
  @return: The IP address of the specified interface.
  """
  s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
  return socket.inet_ntoa(fcntl.ioctl(
      s.fileno(),
      0x8915,  # SIOCGIFADDR
      struct.pack('256s', ifname[:15]))[20:24])

def read_parameter(name, default):
  """
  Get a parameter from the ROS parameter server. If it's not found, a 
  warn is printed.
  @type name: string
  @param name: Parameter name
  @param default: Default value for the parameter. The type should be 
  the same as the one expected for the parameter.
  @return: The restulting parameter
  """
  if not rospy.has_param(name):
    rospy.logwarn('Parameter [%s] not found, using default: %s' % (name, default))
  return rospy.get_param(name, default)
