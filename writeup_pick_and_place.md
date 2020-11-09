## Project: Kinematics Pick & Place
### This is my implementation for Udacity Robotics Software Engineer Nanodegree Program Pick and Place project.

---
**Problem Statement:**

Complete ROS & Gazebo package of Kuka KR210 RRR type robot arm is given in the original [project repository](https://github.com/udacity/RoboND-Kinematics-Project) of Udacity.
Typical Pick and Place operation consist of 6 main steps.  
1. Going for the target objects pose.
2. Approaching to the object
3. Grasping the object
4. Retracting
5. Going for second target pose
6. Release the object

Provided ROS package has all the steps except from 2 and 5. What's left to us is to create a Inverse Kinematics service 
node (IK_Server.py) which calculates the necessary joint angles for all joints for all provided trajectory samples provided by Moveit!? Rest is already done by the ROS package 

**Steps to complete the project:**  


1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code. Detailed explanations about deriving the Forward Kinematics (FK) and Inverse Kinematics (IK) equations are given below. 

**Forward and Inverse Kinematics Calculation Steps:**

Some tasks, even though seems simple to humans, require complex calculations for robots. 
Pick and Place is definitely one of them.  

Here are the solution steps that I followed:

1. Set up the workspace, clone he original repository, run the forward_kinematics.launch file and get familiar with the robot arm.
2. Fill in the Denavit-Hartenberg table by using Rviz and URDF file.
3. Generate homogenous transform matrices for each joint. (from base link to end effector)
4. Generate the Roll-Pitch-Yaw rotation matrix from received pose messages which is the orientation of EE wrt base-link.
5. Calculate the wrist center location (WC) for decoupling the problem into Inverse Position and Inverse Orientation problem.
6. Solve the joint angles for waist, shoulder and elbow joint (theta1-3) by using Inverse Position problem.
7. Solve the joint angles for wrist1, wrist2 and wrist3 joint (theta4-6) by using Inverse Orientation problem.
8. Append all the joint angle results for trajectory points to list and publish the message.

All the details of these steps given below.

**1-Setting up the workspace:**

In this project I used virtual machine provided by the course. As the computer that I am using is quite old, I had some performance issues, but I didn't want to go into all another bother of making a clean Ubuntu 16 and ROS Kinetic install.
 
**2-Denavit-Hartenberg Parameter Table:**

During the lecture most of the table contents are already provided. Detailed inspection of Rviz and URDF file is neded to catch some tricks.
Here is the modified Denavit-Hartenberg table:

Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | 0.75 | qi
1->2 | - pi/2 | 0.35 | 0 | -pi/2
2->3 | 0 | 1.25 | 0 | 0
3->4 |  - pi/2 | -0.054 | 1.5 | 0
4->5 |  pi/2 | 0 | 0 | 0
5->6 | - pi/2 | 0 | 0 | 0
6->EE | 0 | 0 | 0.303 | 0

And here is the figure of the kuka arm drawn by me to make things more visual.

![alt text][image6]


**3-Homogenous Transformation Matrix:**

Here is a general form of homogenous transform matrix. (Image courtesy of Udacity)

![alt text][image1]

<sub>subscript</sub> and <sup>superscript</sup>

Left side of the equation represents the position of point P wrt to Testing A<sub>0</sub> frame

The first term on the right hand side of the equality is a 4x4 matrix called a homogeneous transform; it is composed of four subparts. 
Left side of the 4x4 matrix represent rotation which is R 3x3 rotation matrix. Right side is a 3x1 vector and represents the origin of the B frame relative to the A frame, expressed in terms of the A frame
Rest of the ones and zeros are only for avoiding matrix multiplication dimension problems.

And lastly, last term on the RHS of the equation represents the position of point P wrt to Testing B<sub>0</sub> frame

Homogenous transform matrix can be constructed using modified Denavit-Hartenberg parameters, that is:

![alt text][image2]


Here is how I constructed the homogenous transform between each link for Kuka arm

```python
# Create individual transformation matrices
        T0_1 = Matrix([[cos(q1), -sin(q1), 0, a0],
                       [sin(q1) * cos(alpha0), cos(q1) * cos(alpha0), -sin(alpha0), -sin(alpha0) * d1],
                       [sin(q1) * sin(alpha0), cos(q1) * sin(alpha0), cos(alpha0), cos(alpha0) * d1],
                       [0, 0, 0, 1]])
        T0_1 = T0_1.subs(s)

        T1_2 = Matrix([[cos(q2), -sin(q2), 0, a1],
                       [sin(q2) * cos(alpha1), cos(q2) * cos(alpha1), -sin(alpha1), -sin(alpha1) * d2],
                       [sin(q2) * sin(alpha1), cos(q2) * sin(alpha1), cos(alpha1), cos(alpha1) * d2],
                       [0, 0, 0, 1]])
        T1_2 = T1_2.subs(s)

        T2_3 = Matrix([[cos(q3), -sin(q3), 0, a2],
                       [sin(q3) * cos(alpha2), cos(q3) * cos(alpha2), -sin(alpha2), -sin(alpha2) * d3],
                       [sin(q3) * sin(alpha2), cos(q3) * sin(alpha2), cos(alpha2), cos(alpha2) * d3],
                       [0, 0, 0, 1]])
        T2_3 = T2_3.subs(s)

        T3_4 = Matrix([[cos(q4), -sin(q4), 0, a3],
                       [sin(q4) * cos(alpha3), cos(q4) * cos(alpha3), -sin(alpha3), -sin(alpha3) * d4],
                       [sin(q4) * sin(alpha3), cos(q4) * sin(alpha3), cos(alpha3), cos(alpha3) * d4],
                       [0, 0, 0, 1]])
        T3_4 = T3_4.subs(s)

        T4_5 = Matrix([[cos(q5), -sin(q5), 0, a4],
                       [sin(q5) * cos(alpha4), cos(q5) * cos(alpha4), -sin(alpha4), -sin(alpha4) * d5],
                       [sin(q5) * sin(alpha4), cos(q5) * sin(alpha4), cos(alpha4), cos(alpha4) * d5],
                       [0, 0, 0, 1]])
        T4_5 = T4_5.subs(s)

        T5_6 = Matrix([[cos(q6), -sin(q6), 0, a5],
                       [sin(q6) * cos(alpha5), cos(q6) * cos(alpha5), -sin(alpha5), -sin(alpha5) * d6],
                       [sin(q6) * sin(alpha5), cos(q6) * sin(alpha5), cos(alpha5), cos(alpha5) * d6],
                       [0, 0, 0, 1]])
        T5_6 = T5_6.subs(s)

        T6_7 = Matrix([[cos(q7), -sin(q7), 0, a6],
                       [sin(q7) * cos(alpha6), cos(q7) * cos(alpha6), -sin(alpha6), -sin(alpha6) * d7],
                       [sin(q7) * sin(alpha6), cos(q7) * sin(alpha6), cos(alpha6), cos(alpha6) * d7],
                       [0, 0, 0, 1]])
        T6_7 = T6_7.subs(s)

        T0_7 = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_7
```


**4-Generate Roll-Pitch-Yaw Matrix:**

Position of the end effector is received as a Pose message consist of px, py, pz and quaternions (x,y,z,w) wrt base link.
In the provided IK_server.py file quaternion to euler conversion is already implemented from tf library. 
What needs to be done is to generate a rotation matrix from base link to eef considering the DH and Gazebo differences.
First rotation matrix is generated by create_rrpy_marix function

```python
def create_rrpy_matrix(roll, pitch, yaw):
    # Generation of Rrpy matrix consist of z-y-x extrinsic rotations.
    # Roll, pitch and yaw values are calculated from quaternions which comes from ROS message

    rot_x = rotation_x(roll)
    rot_y = rotation_y(pitch)
    rot_z = rotation_z(yaw)

    Rrpy = rot_z * rot_y * rot_x
    return Rrpy
```

This function takes roll, pitch and yaw values coming from Pose message and converts it into rotation matrix by Z-Y-X extrinsic rotation.
In order to take into account of DH and Gazebo coordinate axis difference one additional conversion is needed.

![alt text][image3]

From diagram above 180 degree rotation on z-axis and then -90 degree rotation on y-axis is applied as Rcorr rotation matrix

```python
# Compensate for rotation discrepancy between DH parameters and Gazebo
# I managed to aling the axes by rotating on z axis by 180 degree and rotating on y axis by -90 degree
Rcorr = rotation_z(pi) * rotation_y(-pi / 2)

# Apply the correction matrix
Rrpy = Rrpy * Rcorr
```
Where rotation_z and rotation_y are predefined functions as:

```python

def rotation_y(radians):
    # Rotation matrix for y-axis

    rotation_matrix = Matrix([
        [cos(radians), 0, sin(radians)],
        [0, 1, 0],
        [-sin(radians), 0, cos(radians)],
    ])
    return rotation_matrix


def rotation_z(radians):
    # Rotation matrix for z-axis

    rotation_matrix = Matrix([
        [cos(radians), -sin(radians), 0],
        [sin(radians), cos(radians), 0],
        [0, 0, 1],
    ])
    return rotation_matrix
```

**5-Calculate Wrist Center Location:**

From the lecture Pick & Place project Section:15

Since we have the case of a spherical wrist involving joints 4,5,6, the position of the wrist center 
is governed by the first three joints. We can obtain the position of the wrist center by using the complete 
transformation matrix we derived in the last section based on the end-effector pose.

For the sake of simplification, let us symbolically define our homogeneous transform as following

![alt text][image4]

w<sub>x</sub> = p<sub>x</sub> - (d<sub>6</sub>+l)*n<sub>x</sub>

w<sub>y</sub> = p<sub>y</sub> - (d<sub>6</sub>+l)*n<sub>y</sub>

w<sub>z</sub> = p<sub>z</sub> - (d<sub>6</sub>+l)*n<sub>z</sub>

This equation is implemented as below.

```python
# Calculating wrist center position wrt base link
#
nx = Rrpy[0, 2]
ny = Rrpy[1, 2]
nz = Rrpy[2, 2]

# d7 = 0.303 which is wc to end effector
wx = px - 0.303 * nx
wy = py - 0.303 * ny
wz = pz - 0.303 * nz
```

**6-Inverse Position Problem:**
Now we have decoupled the problem, theta1, theta2 and theta3 can be calculated in a closed form.

Here is figure I draw for better understanding as it is hard to visualize

![alt text][image7]


Theta1 is the easy one as it is simply tangent angle between wy and wx looking from above
```python
theta1 = atan2(wy, wx)
```

For theta2 and theta3, first calculate the adjacent edge distances.
```python
r = sqrt(wx ** 2 + wy ** 2) - 0.35  # radial distance from link2 to wc from above, 0.35 is a1
A = 1.5011                          # sqrt(1.5 **2 + 0.054 **2)
B = sqrt(r ** 2 + (wz - 0.75) ** 2) #
C = 1.25                            # a2 = 1.25
```

Then apply cosine law to find the angle a.
```python
a = acos((B ** 2 + C ** 2 - A ** 2) / (2 * B * C))
```
Joint 2 angle can be calculated as : pi/2 - red angle a (from cos law) - blue angle(arctan calculated below)
```python
theta2 = pi / 2 - a - atan2(wz - 0.75, r)

```
Similar method is applied for theta 3. First calculate angle b from cosine law.
```python
b = acos((A ** 2 + C ** 2 - B ** 2) / (2 * A * C))
```
Similarly Joint 3 angle can be calculated as pi/2 - red angle b (from cos law) - small drop (arctan calculated below)

```python
# 0.036 radian is necessary to be taken into account for small drop from joint3 to 4. It is simply atan2(0.054,1.5)
theta3 = pi / 2 - (b + 0.036)
```


Great! Inverse Position problem is solved and theta1-3 is calculated.

**7-Inverse Orientation Problem:**
As name states, orientation of end effector will be determined by the angle of the last 3 joints, that is wrist1, wrist2 and wrist3 joint.
To calculate their values let's first find the rotation matrix from base link to link3
FOr that we can use previously derived transformation matrices and use only rotation part.
```python
R0_3 = (T0_1 * T1_2 * T2_3)[0:3, 0:3]
```

Since we know theta1-3 values we can put those values into R0_3 matrix to get the numerical value.
```python
R0_3 = R0_3.evalf(subs={'q1': theta1, 'q2': theta2, 'q3': theta3})
```

```python
R3_6 = R0_3.T * Rrpy  # in theory inverse and transpose of R0_3 is equal as rotation matrix is orthogonal.
```

Now, lets calculate the R3_6 matrix again but this time with symbolic way. This pat is done in IK_debug.py code.
```python
R3_6_sym = simplify((get_rotation_from_transformation_matrix(T3_4) * get_rotation_from_transformation_matrix(T4_5) * \
               get_rotation_from_transformation_matrix(T5_6)).T) * Rcorr
```
where get_rotation_from_transformation_matrix function is simply:

```python
def get_rotation_from_transformation_matrix(transformation_matrix):

    rotation_matrix = transformation_matrix[0:3, 0:3]

    return rotation_matrix
```

Symbolic equation results as:

```python
[-sin(q4)*sin(q6) + cos(q4)*cos(q5)*cos(q6), -sin(q4)*cos(q6) - sin(q6)*cos(q4)*cos(q5), -sin(q5)*cos(q4)],
[                           sin(q5)*cos(q6),                           -sin(q5)*sin(q6),          cos(q5)],
[-sin(q4)*cos(q5)*cos(q6) - sin(q6)*cos(q4),  sin(q4)*sin(q6)*cos(q5) - cos(q4)*cos(q6),  sin(q4)*sin(q5)]]))
```
Now we can use this matrix to calculate the theta4, theta5 and theta6 angles in closed form.

As obvious theta5 is:

```python
theta5 = acos(R3_6[1, 2])
```

For theta4 we can use R3_6[2, 2] and R3_6[0, 2]. Division of these elements 
cancel-out theta5 terms and with a minus signal we have arctan(q4)
```python
theta4 = atan2(-R3_6[2, 2], R3_6[0, 2])
```
For theta6 we can use R3_6[1, 1] and R3_6[1, 0]. Division of these elements 
cancel-out theta5 terms and with a minus signal we have arctan(q6)
```python
theta6 = atan2(R3_6[1, 1], -R3_6[1, 0])
```
Great! Now we have all joint angles!

**8-Append and Publish!**

This part is already ready to go in the provided script. All the trajectory points are collected and above calculations are done for each sampling point inside a for loop.
All joint angles for all trajectory points are appended to a list and published via ROS network.
What I have done is add some print codes to keep track my calculation steps a bit more clear 

```python
print("Theta1, theta2 and theta3 joint angles are calculated")
print("Theta4, theta5 and theta6 joint angles are calculated")
print("Joint angles for eef position", str(x), "are : ", theta1, theta2, theta3, theta4, theta5, theta6)
```

[//]: # (Image References)

[image1]: ./misc_images/homogenous_transform.jpg
[image2]: ./misc_images/homogenous_transform2.jpg
[image3]: ./misc_images/DH_Gazebo_diff.JPG
[image4]: ./misc_images/wrist_center.JPG
[image6]: ./misc_images/DH_Figure.JPG
[image7]: ./misc_images/Inverse_Position.JPG


## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

Please see:

Section 2 - Denavit-Hartenberg Parameter Table

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

Please see:

Section 3 - Homogenous Transformation Matrix
Section 4 - Generate Roll-Pitch-Yaw Matrix

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

Please see:
 
Section-5: Calculate Wrist Center Location
Section-6: Inverse Position Problem
Section-7: Inverse Orientation Problem

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 

1: All the steps for [IK_server.py](./IK_server.py) is given below.
2: Here is the [Video](./video/kuka_pick_and_place.ogg) for successful pick and place operations. 

Here what I have learn:
1: Definitely use a powerful computer. SSD Disk, 4GB ram with 2 processors is incredibly slow, especially while screen recording.
2: I used relatively simple and straight forward method here. For some operations some of the joints seem to be moving unnecessarily much, even though trajectory is on track.
Might be related with joint limits
3: Regarding with first 3 joints, joint configuration is always elbow up as it seem more natural to me. Elbow down option could be also added.
4: For optimizing run time, code can be improved. 
There are some unnecessary calculation which slow he process down. 
As a further step this can be worked. Migrating the calculations to another programming language is also an option(C++)
 





