optitrack
=========

ROS package developed by the [Control Robotics Intelligence Group](http://www.ntu.edu.sg/home/cuong/) from the [Nanyang Technological University, Singapore](http://www.ntu.edu.sg). This package allows to receive in ROS the rigid bodies streamed by Motive 1.7.

### Our setup
  * Optitrack with six Prime 17W cameras.
  * Motive 1.7.5 running on Windows 7, 64 bits.
  * ROS Hydro (Ubuntu 12.04, 64 bits)

**Maintainer:** Francisco Suárez Ruiz, [http://www.romin.upm.es/fsuarez/](http://www.romin.upm.es/fsuarez/)

### Documentation
  * See the installation instructions below.
  * Throughout the various files in this repository.

## Installation

### Basic Requirements

  1. Install [ROS Hydro](http://wiki.ros.org/hydro/Installation/Ubuntu) (**Base Install** Recommended)
```bash
# EXAMPLE FOR UBUNTU 12.04
# Setup your sources.list
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu precise main" > /etc/apt/sources.list.d/ros-latest.list'
#Set up your keys
$ wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | sudo apt-key add -
# Install
$ sudo apt-get update
$ sudo apt-get install ros-hydro-ros-base
``` 
  2. Install the `python-optrix` library:
```
$ sudo pip install optirx
``` 

### Repository Installation

Go to your ROS working directory. e.g.
```
$ cd ~/catkin_ws/src
``` 
Clone this repository:
```
$ git clone https://github.com/crigroup/optitrack.git -b hydro-devel
$ git clone https://github.com/gt-ros-pkg/hrl-kdl.git -b hydro
``` 
Install any missing dependencies using rosdep:
```
$ rosdep update
$ rosdep install --from-paths . --ignore-src --rosdistro hydro -y
``` 
Now compile your ROS workspace. e.g.
```
$ cd ~/catkin_ws && catkin_make
``` 

### Testing Installation

Be sure to always source the appropriate ROS setup file, e.g:
```
$ source ~/catkin_ws/devel/setup.bash
``` 
You might want to add that line to your `~/.bashrc`

Try the following commands (one per terminal) to start a joint trajectory example:
```
$ roslaunch optitrack optitrack_pipeline.launch
$ rostopic echo /optitrack/rigid_bodies
``` 

## Changelog
### 0.1.0 (2015-10-20)
* Initial Release
