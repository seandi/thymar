# Thymar project

## Requirements

- thymio_description package (already intalled):
  1. git clone https://github.com/jeguzzi/ros-aseba.git
  2. copy the thymio_description folder to the /src folder
  3. build the package with catkin build
  
- velodyne plugin (to be installed):
  1. move to catkin_ws/src/
  2. git clone https://bitbucket.org/DataspeedInc/velodyne_simulator.git
  3. catkin build
  
- ros_pcl (to be installed):
  1. move to catkin_ws/src/
  2. git clone https://github.com/ros-perception/perception_pcl.git
  3. catkin build
  
## How to Install
To install the packages
1. move to catkin_ws/src/
2. git clone https://github.com/seandi/thymar.git
3. catkin build

## Overview
The repo contains the ros packages for the Thymar robot. Inside the thymar folder there are two packages: 
1. the thymar_description is the ros package containing all the models and launch file for similating in Gazebo the Thymar robot
2. the thymar package contains the scripts for controlling the robot, see [readme](thymar/README.md)

### How to run
The gazebo simulation of the robot can be launched as follows
`roslaunch thymar_description thymar_gazebo_bringup.launch name:=thymar world:=empty`
then launch the node processing the lidar pointcloud:
`roslaunch thymar_lidar lidar_processor.launch name:=thymar`
and finally launch the main script:
`rosrun thymar Thymar.py _name:=thymar`
NOTE: cuurently there is a bug for which if the target is immediately visible when the lidar node is started, the Thymar.py script may never receive its position.

## Visualizing the pointcloud
To visualize the point cloud from the Thymar:
1. open a new terminal and start `rviz`
2. change *fixed frame* from *map* to *name_of_the_robot/velodyne*
3. click add -> scroll down to PointCloud2 
4. under pointcloud2 set topic=*name_of_the_robot/velodyne_points*
