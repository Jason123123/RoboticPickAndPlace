## Project: Kinematics Pick & Place
### Writeup Template: You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

---


**Steps to complete the project:**  


1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code. 


[//]: # (Image References)

[image1]: ./misc_images/DH.png
[image2]: ./misc_images/triangle.png
[image3]: ./misc_images/misc2.png
[image4]: ./misc_images/R36.png
[image5]: ./misc_images/1.png
[image6]: ./misc_images/2.png
[image7]: ./misc_images/3.png
[image8]: ./misc_images/pick.png
[image9]: ./misc_images/4.png
[image10]: ./misc_images/5.png
[image11]: ./misc_images/6.png
[image12]: ./misc_images/7.png

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

The urdf file contains detailed parameters for each links and joints.

In general, each transform would require six independent parameters to describe. But DH scheme uses 4 parameters. Their meanings are as follows:

1. alpha: twist angle between z-axis
2. a: distance between z-aixs
3. d: link offset measured along x-axis
4. theta: angle between x-axis

Parameters:

1. d1: Is calculated summing 0.33 and 0.42 that are the heights of joint 1 and 2 from the ground, which is equal to the distance of O1 and ground.
2. a1 equals to the x coordinate of O2 in urdf file.
3. a2 equals to the z coordinate of O3 in urdf file.
4. a3 equals to the disparity of O4 and O5.
5. d4 is the sum of x coordinate of O4 and O5(0.96 + 0.54).
6. d6 is the sum of x coordinate of O6 and O_G(0.11 + 0.193).

![alt text][image1]

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

Links | alpha(i-1) | a(i-1) | d(i) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | 0.75 | q1
1->2 | - pi/2 | 0.35 | 0 | -pi/2 + q2
2->3 | 0 | 1.25 | 0 | q3
3->4 |  -pi/2 | 0.054 | 1.5 | q4
4->5 | pi/2 | 0 | 0 | q5
5->6 | -pi/2 | 0 | 0 | q6
6->EE | 0 | 0 | 0.303 | 0

I used a function to calculate the transformation matrix of each joint:

```python
def TF_Matrix(alpha, a, d, q):
            TF = Matrix([[          cos(q),           -sin(q),           0,             a],
                        [sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                        [sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
                        [                0,                 0,          0,              1]])
        return TF
```

By substituting matrix elements with DH parameters, each transformation matrix could be written as follows:

```python
T0_1 = [[cos(q1), -sin(q1), 0, 0],
        [sin(q1), cos(q1), 0, 0],
        [0, 0, 1, 0.750000000000000],
        [0, 0, 0, 1]]

T1_2 = [[sin(q2), cos(q2), 0, 0.350000000000000],
        [0, 0, 1, 0],
        [cos(q2), -sin(q2), 0, 0],
        [0, 0, 0, 1]]

T2_3 = [[cos(q3), -sin(q3), 0, 1.25000000000000],
        [sin(q3), cos(q3), 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]]

T3_4 = [[cos(q4), -sin(q4), 0, -0.0540000000000000],
        [0, 0, 1, 1.50000000000000],
        [-sin(q4), -cos(q4), 0, 0],
        [0, 0, 0, 1]]

T4_5 = [[cos(q5), -sin(q5), 0, 0],
        [0, 0, -1, 0],
        [sin(q5), cos(q5), 0, 0],
        [0, 0, 0, 1]]

T5_6 = [[cos(q6), -sin(q6), 0, 0],
        [0, 0, 1, 0],
        [-sin(q6), -cos(q6), 0, 0],
        [0, 0, 0, 1]]

T6_EE = [[1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, 0.303000000000000],
        [0, 0, 0, 1]]
```

The transformation between base link and gripper link could be calculated as follows:
```
T0_EE = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_EE
```
#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

After we get the position of end effector, we could calculate the position of the wrist center.
```
WC = EE - (0.303) * ROT_EE[:,2]
```
Angle theta1 is relatively easy to compute using wrist center position.
```
theta1 = atan2(WC_y, WC_x)
```
![alt text][image2]

The calculation of theta2 and theta3 is trickier. To calculate theta2 and theta3, we first calculate angle a and b. the length of A and C are already known, the length of B could be computed by taking the square root of (WC_x^2 + WC_y^2).

After we have the length of side A, B, C, the value of angle a, b and c could be calculated using Cosine Laws:
```
angle_a = acos((side_b * side_b + side_c * side_c - side_a * side_a) / (2 * side_b * side_c))
```

Then we calculate theta2 and theta3:
```
theta2 = pi/2 - angle_a - atan2((Wz - 0.75), sqrt(Wx**2 + Wy**2) - 0.35)
```
```
theta3 = pi/2 - angle_b + 0.036
```

For deriving theta 4,5,6, I used equations provided in the euler angles of rotation matrix section. By multiplying R0_3 and R0_6, we can derive R3_6. Based on R3_6, theta4,5,6 can be calculated.

Each element in R3_6 has following meanings:
![image4]
Based on these equations, theta values can be easily computed.

![image5]
![image6]
![image7]


```
theta4 = atan2(R3_6[2,2], -R3_6[0,2])
```
```
theta5 = atan2(sqrt(R3_6[0,2]*R3_6[0,2] + R3_6[2,2]*R3_6[2,2]), R3_6[1,2])
```
```
theta6 = atan2(-R3_6[1,1], R3_6[1,0])
```
### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 


The IK_server.py first calculate the transformation matrices of each joint, then solve the end effector positions and get the value for theta1 to theta6.

To reduce computation costs, rotation matrix x, y and z are calculated outside the main loop in `handle_calculate_IK()` function.

I followed the instructions of the walk through video. Most of the time the robot arm could move to the object location correctly. But in some cases, the gripper may tip down the cylinder.

![image8]

The images below shows the moving trajactory:
![image9]
![image10]
![image11]

My program succeed 8/10 times performing a complete pick and place operation.

![image12]

#### 2. Further Improvement

Although the robot arm could finish most of the pick up tasks successfully, there are still some cases where the gripper would tip down the target object and cause failure.

To solve this problem, maybe we can change the gripper parameters to constrain the gripper into a smaller space range.


