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
  
## Install packages
To install the packages
1. move to catkin_ws/src/
2. git clone https://github.com/seandi/thymar.git
3. catkin build

## Overview
The repo contains the ros packages for the Thymar robot. Inside the thymar folder there are two packages: 
1. the thymar_description is the ros package containing all the models and launch file for similating in Gazebo the Thymar robot
2. the thymar package contains the scripts for controlling the robot

### The Thymar robot model 
The gazebo simulation of the robot can be launched as follows
`roslaunch thymar_description thymar_gazebo_bringup.launch name:=thymar world:=empty`
