# walker

## Overview

This is a ROS package that implements obstacle avoidance on turtlebot as an excersise to
demonstrate the use of gazebo, rosbag, c++11 and good programming practices. there is one node
in this packeg, walker_node, it subscribes to /scan to read laser data, and commands turtlebot
over  /mobile_base/commands/velocity topic. 

# Install and Build instructions

## Make catkin workspace
in a terminal type:
```
mkdir -p catkin_ws/src

```
## Initialize catkin workspace
```
cd ~/catkin_ws/src
catkin_init_workspace
```
## Clone package
git clone https://github.com/royneal/walker.git

## Build Workspace 
in the same terminal:
```
cd ~/catkin_ws
catkin_make 
```
## Launching Gazebo and Walker
```
cd ~/catkin_ws
roslaunch walker walker.launch broadcast:=1 rate:=20 record_cmd:=0

```

## Rosbag
to start simulation with rosbag recording enabled:
in a  terminal 
```
cd ~/catkin_ws
roslaunch walker walker.launch broadcast:=1 rate:=20 record_cmd:=1
```
where "record_cmd:=1" enables recording and "record_cmd:=0" disables it. 

## Rosbag Play

close all terminals to terminate gazebo and walker_node.
start a new terminal and start roscore
```
roscore
```
Open a second terminal 
```
rosbag play ./results/2018-11-18-22-51-13.bag
```
Open another new terminal again and check the recorded data
```
rostopic echo /scan
```
or to view motor commands : 
```
rostopic echo /mobile_base/commands/velocity 
```

## Dependencies

ROS kinetic
