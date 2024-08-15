# TurtleBot3 SLAM Simulation and Navigation

## Overview

Navigation in robots in the SLAM simulation with the TurtleBot3 in Gazebo, various environments and robot models can be selected or created for the virtual world. Once the simulation environment is prepared and the robot is ready, the SLAM process closely mirrors that of using a real TurtleBot3. 

## Map Creation and Navigation

1. **Create and Save Map**: After successfully creating a map, save it and run the navigation node.
2. **Initial Pose Estimation**: It is essential to perform initial pose estimation before running the navigation. This step initializes the Adaptive Monte Carlo Localization (AMCL) parameters crucial for precise navigation.
3. **Robot Positioning**: The TurtleBot3 must be accurately positioned on the map, with the sensor data from the Laser Distance Sensor (LDS) aligning well with the displayed map.
4. **Setting Destination**: To set a destination for the robot, click on the map and drag the green arrow to indicate the desired direction. The base of the arrow represents the x, y coordinates of the destination, while the orientation of the arrow determines the angle θ. Once the coordinates and angle are set, the TurtleBot3 will begin moving toward the destination immediately.

## Python Script for Navigation

A Python script has been developed to control the TurtleBot3 using ROS (Robot Operating System) and the move_base action server. This script enables the input of multiple goal coordinates (x, y) and orientations (w), allowing the robot to navigate to specified targets. The script performs the following tasks:

- Initializes the ROS node.
- Creates an action client to interact with the move_base server.
- Sends navigation goals to the robot.




