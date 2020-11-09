# RoboND-Pick-and-Place
This is my implementation for Udacity Robotics Software Engineer Nanodegree Program Pick and Place project.

###Problem Statement

Complete ROS & Gazebo package of Kuka KR210 RRR type robot arm is given in the original [project repository](https://github.com/udacity/RoboND-Kinematics-Project) of Udacity.
Typical Pick and Place operation consist of 6 main steps.  
1. Going for the target objects pose.
2. Approaching to the object
3. Grasping the object
4. Retracting
5. Going for second target pose
6. Release the object

Provided ROS package has all the steps except from 2 and 5. What's left to us is to create a Inverse Kinematics service 
node (IK_Server.py) which calculates the necessary joint angles for all joints for all provided trajectory samples provided by Moveit!?. Rest is already done by the ROS package 

###Description
1-[writeup_pick_and_place.md](writeup_pick_and_place.md) Writeup file includes solution steps and all explanations
2-[IK_server.py](./IK_server.py) : Rosnode for Forward and Inverse Kinematic Calculations
3-[IK_debug.py](./IK_debug.py) : Python script for debugging and additional calculations.
4-[Video](./video/kuka_pick_and_place.ogg): Video of successful pick and place operations.

###My Comments
1-
2- 