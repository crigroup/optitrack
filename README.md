optitrack
=========
[![Build Status](https://travis-ci.org/crigroup/optitrack.png?branch=master)](https://travis-ci.org/crigroup/optitrack)

ROS package developed by the [Control Robotics Intelligence Group](http://www.ntu.edu.sg/home/cuong/) from the [Nanyang Technological University, Singapore](http://www.ntu.edu.sg). This package allows to receive in ROS the rigid bodies streamed by Motive 1.7.

The **master** branch is compatible with:
  * ROS Indigo
  * ROS Kinetic

### Maintainer
[Francisco Suárez-Ruiz](fsuarez6.github.io)

### Our setup
  * Optitrack with six Prime 17W cameras.
  * Motive 1.7.5 running on Windows 7, 64 bits.
  * ROS Kinetic (Ubuntu 16.04, 64 bits)

### Documentation
  * See the installation instructions below.
  * Throughout the various files in this repository.

# Install
The following instructions have been tested in Ubuntu 16.04 (Xenial), 64 bits.

## ROS Kinetic
Set-up your computer to accept software from packages.ros.org and set-up your
keys following the steps described at
http://wiki.ros.org/kinetic/Installation/Ubuntu

Now, install the ROS bare bones:
```bash
# Installation
sudo apt-get update
sudo apt-get install python-wstool ros-kinetic-ros-base
# Initialize rosdep
sudo rosdep init
rosdep update
```

## ROS Package installation
Install the `python-optrix` module:
```
pip install optirx --user
```

Go to your ROS working directory:
```
cd ~/catkin_ws/src
```

Clone this repository:
```
git clone git clone https://github.com/crigroup/optitrack.git
```

Install any missing dependencies using rosdep:
```
rosdep update
rosdep install --from-paths . --ignore-src -y
```

Now, compile your ROS workspace:
```
cd ~/catkin_ws && catkin_make
```

## Testing the Installation
Make sure you always source the appropriate ROS setup file, e.g:
```
$ source ~/catkin_ws/devel/setup.bash
```
You might want to add that line to your `~/.bashrc`

Try the following commands (one per terminal):
```
roslaunch optitrack optitrack_pipeline.launch iface:=eth1
rostopic echo /optitrack/rigid_bodies
```

# Troubleshooting

### NotImplementedError: Force plate data not supported

Check the version of your Motive SDK: go to _Help -> About Motive... -> NatNet Streaming Module_. It can be something like `2.7.0.0`.

Then, in `rigid_body_publisher.py` change this line:
```python
version = (2, 9, 0, 0) # the latest SDK version
```
To this:
```python
version = (2, 7, 0, 0) # Our SDK version
```

### Don't use the netgear router for your ROS network

You should have two independent networks:
* One for the Optitrack cameras + windows computer
* One for windows computer + ROS computer

This implies that your windows computer needs 2 network cards. Laptops usually
have 2, one wired and one wireless.

### The windows firewall isn't your friend

If your ROS network appears as _Public_ in the windows computer, the streaming will be blocked by the firewall. You need to make this network **Private** ([How to make a public network private windows?](http://bfy.tw/HTs6))

### Wrong subnet mask

A mismatch between subnet masks could lead to errors. Use the same subnet mask in both computers. E.g: `255.255.255.0`

### Have you created the Rigid Bodies in Motive?

This driver has been only tested with Rigid Bodies. You can create them in Motive as follows:
Select the markers -> Right click -> _Rigid Body -> Create From Selected Markers_

### Is Motive streaming?

Set _Multicast Interface_ field to the IP address of your ROS computer
_View -> Data Streaming -> Advanced Network Options -> Multicast Interface_

### Mutual ping

Check you can ping in both directions: windows computer <-> ROS computer

### Still doesn't work

* We have Motive 1.7.5 running on Windows 7, 64 bits and sometimes we need to
_reset_ the local interface for the streaming to work. This seems to be a bug.
The _reset_ is done by changing other interface and then back to the interface
you want to use: _View -> Data Streaming -> Network Interface -> Local
interface_
