## Project: Kinematics Pick & Place
### Writeup Template: You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

---


<!-- **Steps to complete the project:**  


1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code.  -->


[//]: # (Image References)

[image1]: ./images/Selection_079.png
[image2]: ./images/Selection_081.png
[image3]: ./misc_images/misc2.png
[image4]: ./images/dh_params.png
[image5]: ./images/four_transforms.png
[image6]: ./images/figure.png
[image7]: ./images/T0_1.png
[image8]: ./images/T1_2.png
[image9]: ./images/T2_3.png
[image10]: ./images/T3_4.png
[image11]: ./images/T4_5.png
[image12]: ./images/T5_6.png
[image13]: ./images/T6_7.png


## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points 

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

The Kuka KR210 has 6 revolute joints.

![alt text][image1]

DH parameters can be derived from analysis of the KR210's joint measurements

![alt text][image6]

![alt text][image4]


#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

Four Transforms:
![alt text][image5]

Individual Transforms:

T0_1
![alt text][image7]

T1_2
![alt text][image8]

T2_3
![alt text][image9]

T3_4
![alt text][image10]

T4_5
![alt text][image11]

T5_6
![alt text][image12]

T6_7
![alt text][image13]

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.


theta1 = atan2(WC[1], WC[0])

side_a = 1.501
side_b = sqrt(pow(sqrt(WC[0] * WC[0] + WC[1] * WC[1]) - 0.35, 2)+ pow((WC[2] - 0.75), 2))
side_c = 1.25

angle_a = acos((side_b * side_b + side_c * side_c - side_a * side_a) / (2 * side_b * side_c))
angle_b = acos((side_a * side_a + side_c * side_c - side_b * side_b) / (2 * side_a * side_c))
angle_c = acos((side_a * side_a + side_b * side_b - side_c * side_c ) / (2 * side_a * side_b))

theta2 = pi/2 - angle_a - atan2(WC[2] - 0.75, sqrt(WC[0] * WC[0] + WC[1] * WC[1]) - 0.35)
theta3 = pi/2 - (angle_b + 0.036)

R0_3 = T0_1[0:3,0:3] * T1_2[0:3,0:3] * T2_3[0:3,0:3]
R0_3 = R0_3.evalf(subs={q1: theta1, q2:theta2, q3: theta3})
R3_6 = R0_3.transpose() * ROT_EE

theta4 = atan2(R3_6[2,2], -R3_6[0,2])
theta5 = atan2(sqrt(R3_6[0,2]*R3_6[0,2] + R3_6[2,2]*R3_6[2,2]), R3_6[1,2])
theta6 = atan2(-R3_6[1,1], R3_6[1,0])    


### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 


The IK_server.py and IK_debug.py files contain the bulk of my code in addition to the gazebo_grasp_plugin, kuka_arm, and moveit. With the current IK solve the robot successfully completes 8/10 pick and place cycles, however there is room for optimization and improvement as the movements are yet streamlined and the failure rate is around 10% (knocking over object when picking or prematurely dropping).

![kuka-gazebo-gif](https://github.com/WolfeTyler/Kuka-Arm-Robotics-Challenge-Project/blob/master/images/gazebo-demo.gif)

I posted a successful pick and place on youtube here:

[![Youtube Video](http://img.youtube.com/vi/odLVMeGWJ18/0.jpg)](https://youtu.be/R3zy9lVtCY0)


