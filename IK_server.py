#!/usr/bin/env python

# Copyright (C) 2017 Udacity Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *


def rotation_x(radians):
    #Rotation matrix for x-axis

    rotation_matrix = Matrix([
        [1, 0, 0],
        [0, cos(radians), -sin(radians)],
        [0, sin(radians), cos(radians)],
    ])
    return rotation_matrix


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


def create_rrpy_matrix(roll, pitch, yaw):
    # Generation of Rrpy matrix consist of z-y-x extrinsic rotations.
    # Roll, pitch and yaw values are calculated from quaternions which comes from ROS message

    rot_x = rotation_x(roll)
    rot_y = rotation_y(pitch)
    rot_z = rotation_z(yaw)

    Rrpy = rot_z * rot_y * rot_x
    return Rrpy


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print
        "No valid poses received"
        return -1
    else:

        # Create symbols
        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')

        # Create Modified DH parameters
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
        #
        # Define Modified DH Transformation matrix
        s = {alpha0: 0, a0: 0, d1: 0.75,  # row 1
             alpha1: -pi / 2, a1: 0.35, d2: 0, q2: q2 - pi / 2,  # row 2
             alpha2: 0, a2: 1.25, d3: 0,  # row 3
             alpha3: -pi / 2, a3: -0.054, d4: 1.5,  # row 4
             alpha4: pi / 2, a4: 0, d5: 0,  # row 5
             alpha5: -pi / 2, a5: 0, d6: 0,  # row 6
             alpha6: 0, a6: 0, d7: 0.303, }  # row 7
        #
        # Create individual transformation matrices, better to do this outside of the loop
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

        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

            # Extract end-effector position and orientation from request
            # px,py,pz = end-effector position
            # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                 req.poses[x].orientation.z, req.poses[x].orientation.w])

            # Generate Rrpy matrix for end effector wrt base link
            Rrpy = create_rrpy_matrix(roll, pitch, yaw)

            # Compensate for rotation discrepancy between DH parameters and Gazebo
            # I managed to aling the axes by rotating on z axis by 180 degree and rotating on y axis by -90 degree
            Rcorr = rotation_z(pi) * rotation_y(-pi / 2)

            # Apply the correction matrix
            Rrpy = Rrpy * Rcorr

            # Calculating wrist center position wrt base link
            #
            nx = Rrpy[0, 2]
            ny = Rrpy[1, 2]
            nz = Rrpy[2, 2]

            # d7 = 0.303 which is wc to end effector
            wx = px - 0.303 * nx
            wy = py - 0.303 * ny
            wz = pz - 0.303 * nz

            # Calculate joint angles using Geometric IK method

            ### Calculate theta1 - theta3 - Inverse Position Problem
            # theta 1 is the easy one as it is simply tangent angle between wy and wx looking from above

            theta1 = atan2(wy, wx)

            # theta 2 and theta 3 is relatively tricky as it is hard to visualize. For both ve need to use
            # trigonometric calculations for two adjacent triangle

            r = sqrt(wx ** 2 + wy ** 2) - 0.35  # radial distance from link2 to wc from above, 0.35 is a1

            # Construct the triangle for cosine law. A is the angle in front of corner with angle a,
            # B is the angle in front of corner with angle b. For more detail, see writeup.md

            A = 1.5011                          # sqrt(1.5 **2 + 0.054 **2)
            B = sqrt(r ** 2 + (wz - 0.75) ** 2) #
            C = 1.25                            # a2 = 1.25

            # calculate angle a by using cosine law
            a = acos((B ** 2 + C ** 2 - A ** 2) / (2 * B * C))

            # compensate 90 degree resting angle and the vertical difference between link2 and wc
            theta2 = pi / 2 - a - atan2(wz - 0.75, r)

            # calculate angle b by using cosine law
            b = acos((A ** 2 + C ** 2 - B ** 2) / (2 * A * C))

            # compensate 90 degree resting angle and the small drop difference between link3 and wc
            # 0.036 radian is necessary to be taken into account for small drop from joint3 to 4. It is simply atan2(0.054,1.5)
            theta3 = pi / 2 - (b + 0.036)

            print("Theta1, theta2 and theta3 joint angles are calculated")

            ### Calculate theta4 - theta6 - Inverse Orientation Problem

            #Rotation matrix from base link to link 3 derived from transformation matrix
            R0_3 = (T0_1 * T1_2 * T2_3)[0:3, 0:3]

            # Apply the calculated theta1 - theta 3 values into rotation matrix.
            R0_3 = R0_3.evalf(subs={'q1': theta1, 'q2': theta2, 'q3': theta3})

            #Calculate the rotation matrix from link3 to link 6 and apply the correction matrix

            R3_6 = R0_3.T * Rrpy  # in theory inverse and transpose of R0_3 is equal as rotation matrix is orthogonal.
            # But sometimes inverse method seems to be causing some numerical problems so I used transpose method.

            # each element of R3_6 matrix consist of one or more trigonometric terms.
            # Derivation of theta4,5 and 6 from these terms are well explained in the writeup document.
            # Simply, first I calculated theta 5 by acos function as, R3_6[1, 2] = cos(q5)
            # Then theta4 and 6 are calculated by dividing to terms of the matrix to cancel-out unwanted angle terms.

            theta5 = acos(R3_6[1, 2])

            if sin(theta5) < 0:
                theta4 = atan2(-R3_6[2, 2], R3_6[0, 2])
                theta6 = atan2(R3_6[1, 1], -R3_6[1, 0])
            else:
                theta4 = atan2(R3_6[2, 2], -R3_6[0, 2])
                theta6 = atan2(-R3_6[1, 1], R3_6[1, 0])

            print("Theta4, theta5 and theta6 joint angles are calculated")
            print("Joint angles for eef position", str(x), "are : ", theta1, theta2, theta3, theta4, theta5, theta6)

            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
            joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
            joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print
    "Ready to receive an IK request"
    rospy.spin()


if __name__ == "__main__":
    IK_server()
