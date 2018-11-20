# Zoomba_Roomba_Robot
[![MIT licensed](https://img.shields.io/badge/license-MIT-green.svg)](https://github.com/harshkakashaniya/zoomba_roomba_robot/blob/master/LICENSE)

## Overview

A small project of ROS with the use of Gazebo and turtle bot. Here we implemented algorithm of automatic vaccum cleaner which does floor cleaning automatically. If there is obstacle around it changes the path without colliding. Hence system moves in random ways and cleans whole floor without bot being damaged.   

## License
```
MIT License

Copyright (c) 2018 Harsh Kakashaniya

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```

## Dependencies
### Install ROS
ROS should be installed on the system. This package is tested on Ubuntu 16.04 LTS with [ROS Kinetic Distribution](http://wiki.ros.org/kinetic).
Installation Instructions can be found [here](http://wiki.ros.org/kinetic/Installation).
- [Turtlebot](https://www.turtlebot.com/) packages are required. Run the following command to install all turtlebot related packages.
```
sudo apt-get install ros-kinetic-turtlebot*
```
- [Gazebo](http://wiki.ros.org/gazebo_ros_pkgs) version 7.0.0 or above. Installation instructions can be found [here](http://gazebosim.org/tutorials?cat=guided_b&tut=guided_b1).

### Install catkin
catkin is a Low-level build system macros and infrastructure for ROS.
catkin is included by default when ROS is installed. But, it can also be installed with apt-get

```
sudo apt-get install ros-kinetic-catkin
```

### Creating a catkin workspace
Create a catkin workspace using following instructions:
```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
$ git clone --recursive https://github.com/harshkakashaniya/zoomba_roomba_robot
$ catkin_create_pkg zoomba_roomba_robot
```
### Build the program
```
cd ~/catkin_ws
catkin_make
```
catkin_make works as cmake and make of our Cmake process. After running this we will see two more folders named build and devel

### Run the program

## In Terminal
```
cd ~/catkin_ws
source devel/setup.bash
roslaunch zoomba_roomba_robot zoomba.launch Record:=true
```
by this command we will have Walker.bag file in result folder. One such sample file is kept in result/Walker.bag

## Disable bag file recording,
To Disable bag file recording, come to terminal
```
Press Ctrl+C
```
## Inspecting the bag file
to check what are the details of any particular bag file.

```
roscore
```
First command if roscore not already working else launch file should be working. 
```
cd ~catkin_ws/
rosbag info src/zoomba_roomba_robot/results/SampleWalker.bag

```
We will see similar ouput
```
path:        results/SampleWalker.bag
version:     2.0
duration:    29.4s
start:       Dec 31 1969 19:00:00.29 (0.29)
end:         Dec 31 1969 19:00:29.74 (29.74)
size:        11.5 MB
messages:    22145
compression: none [15/15 chunks]
types:       bond/Status                           [eacc84bf5d65b6777d4c50f463dfb9c8]
             dynamic_reconfigure/Config            [958f16a05573709014982821e6822580]
             dynamic_reconfigure/ConfigDescription [757ce9d44ba8ddd801bb30bc456f946f]
             gazebo_msgs/LinkStates                [48c080191eb15c41858319b4d8a609c2]
             gazebo_msgs/ModelStates               [48c080191eb15c41858319b4d8a609c2]
             geometry_msgs/Twist                   [9f195f881246fdfa2798d1d3eebca84a]
             nav_msgs/Odometry                     [cd5e73d190d741a2f92e81eda573aca7]
             rosgraph_msgs/Clock                   [a9c97c1d230cfc112e270351a944ee47]
             rosgraph_msgs/Log                     [acffd30cd6b6de30f120938c17c593fb]
             sensor_msgs/Imu                       [6a62c6daae103f4ff57a132d6f95cec2]
             sensor_msgs/JointState                [3066dcd76a6cfaef579bd0f34173e9fd]
             sensor_msgs/LaserScan                 [90c7ef2dc6895d81024acba2ac42f369]
             std_msgs/String                       [992ce8a1687cec8c8bd883ec73ca41d1]
             tf2_msgs/TFMessage                    [94810edda583a504dfda3829e70d7eec]
topics:      /clock                                            2949 msgs    : rosgraph_msgs/Clock                  
             /cmd_vel_mux/active                                  1 msg     : std_msgs/String                      
             /cmd_vel_mux/input/teleop                           59 msgs    : geometry_msgs/Twist                  
             /cmd_vel_mux/parameter_descriptions                  1 msg     : dynamic_reconfigure/ConfigDescription
             /cmd_vel_mux/parameter_updates                       1 msg     : dynamic_reconfigure/Config           
             /depthimage_to_laserscan/parameter_descriptions      1 msg     : dynamic_reconfigure/ConfigDescription
             /depthimage_to_laserscan/parameter_updates           1 msg     : dynamic_reconfigure/Config           
             /gazebo/link_states                               2942 msgs    : gazebo_msgs/LinkStates               
             /gazebo/model_states                              2942 msgs    : gazebo_msgs/ModelStates              
             /gazebo/parameter_descriptions                       1 msg     : dynamic_reconfigure/ConfigDescription
             /gazebo/parameter_updates                            1 msg     : dynamic_reconfigure/Config           
             /gazebo_gui/parameter_descriptions                   1 msg     : dynamic_reconfigure/ConfigDescription
             /gazebo_gui/parameter_updates                        1 msg     : dynamic_reconfigure/Config           
             /joint_states                                     2823 msgs    : sensor_msgs/JointState               
             /laserscan_nodelet_manager/bond                     56 msgs    : bond/Status                           (2 connections)
             /mobile_base/commands/velocity                      56 msgs    : geometry_msgs/Twist                  
             /mobile_base/sensors/imu_data                     2823 msgs    : sensor_msgs/Imu                      
             /mobile_base_nodelet_manager/bond                  112 msgs    : bond/Status                           (3 connections)
             /odom                                             2822 msgs    : nav_msgs/Odometry                    
             /rosout                                            326 msgs    : rosgraph_msgs/Log                     (10 connections)
             /rosout_agg                                        309 msgs    : rosgraph_msgs/Log                    
             /scan                                              276 msgs    : sensor_msgs/LaserScan                
             /tf                                               3640 msgs    : tf2_msgs/TFMessage                    (2 connections)
             /tf_static                                           1 msg     : tf2_msgs/TFMessage

```


## Playing back the bag file following will be the output
```
roscore
```
First command if roscore not already working
```
cd catkin_ws/
source devel/setup.bash
rosbag play src/zoomba_roomba_robot/results/SampleWalker.bag
```
Similar is the output of Rosbag when the above code is run.
```
[ INFO] [1542738013.410444920]: Opening src/zoomba_roomba_robot/results/SampleWalker.bag

Waiting 0.2 seconds after advertising topics... done.

Hit space to toggle paused, or 's' to step.
 [PAUSED ]  Bag Time:     20.480240   Duration: 20.190240 / 29.450000               8013.35


```

## Termination
Press Ctrl+C in all the terminals to close the running program.

## OR

In new terminal type
```
rosnode kill talker
rosnode kill listener
```
