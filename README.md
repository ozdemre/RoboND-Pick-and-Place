# Udacity Robotics Software NanoDegree Program Pick&Place Project
This is my implementation for Udacity Robotics Software Engineer Nanodegree Program Pick and Place project.

**Problem Statement:**

Complete ROS & Gazebo package of Kuka KR210 RRR type robot arm is given in the original [project repository](https://github.com/udacity/RoboND-Kinematics-Project) of Udacity.
Typical Pick and Place operation consist of 6 main steps.  
1. Going for the target objects pose.
2. Approaching to the object
3. Grasping the object
4. Retracting
5. Going for second target pose
6. Release the object



Provided ROS package has all the steps except from 1 and 5. What's left to us is to create a Inverse Kinematics service 
node (IK_Server.py) which calculates the necessary joint angles for all joints for all provided trajectory samples provided by Moveit! Rest is already done by the ROS package 

**How to use this code?**

1. Go to Udacity original repo, clone and follow the instructions
2. Go to the `/src/RoboND-Kinematics-Project/kuka_arm/launch` folder and change the demo flag to "false" in the `inverse_kinematics.launch` file
3. Replace the IK_server.py file under `/src/RoboND-Kinematics-Project/kuka_arm/scripts` with [IK_server.py](./IK_server.py) file
4. Change execution permissions

```bash
$ cd ~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/scripts
sudo chmod u+x IK_server.py
```
5. Open a new terminal, source your workspace and execute:

```bash
$ cd ~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/scripts
$ ./safe_spawner.sh 
```
This will initialize Moveit, Rviz and Gazebo ready for Pick and Place operation.

6. Open another terminal, source it and run the IK_server.py
```bash
$ cd ~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/scripts
$ rosrun kuka_arm IK_server.py   
```

**Description of Files**
1. [writeup_pick_and_place.md](writeup_pick_and_place.md) Writeup file includes solution steps and all explanations
2. [IK_server.py](./IK_server.py) : ROS node for Forward and Inverse Kinematic Calculations
3. [IK_debug.py](./IK_debug.py) : Python script for debugging and additional calculations.
4. [Video](https://www.youtube.com/watch?v=30WFnx8ArHI): Video of 9/10 successful pick and place operations. 



