# Thymar controller

## Overview
The script `Thymar.py` implements the main interface for the robot, it takes care of periodically reading all the data from the sensors by subsricing to the ROS topic and publishes the velocity messages to control the robot. The `run()` method loops indefinitely and calls a controller function that receives as inputs all the data from the robot and computes the appropriate velocities for controlling it. 

## Run the demo
`rosrun thymar Thymar.py _name:=thymar`
