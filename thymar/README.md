# Thymar controller

## Overview
The script `Thymar.py` implements the main interface for the robot, it takes care of periodically reading all the data from the sensors by subsricing to the ROS topic and publishes the velocity messages to control the robot. The `run()` method loops indefinitely and calls a controller function that receives as inputs all the data from the robot and computes the appropriate velocities for controlling it. 

All the other scripts are taken from the second lab and implement the logic for the random exploration and are used for now as a demo.

The reading of the point cloud is missing.

## Run the demo
`rosrun thymar Thymar.py _name:=thymar`
