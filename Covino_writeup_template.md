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

[image1]: ./Pickup.png
[image2]: ./Move_1.png
[image3]: ./Move_2.png
[image4]: ./Drop.png
[image5]: ./misc_images/misc3.png
[image6]: ./DH_table.png
[image7]: ./DH_Table_TF_Matrix.png
[image8]: ./TF_Matrix_Function.png
[image9]: ./Total_Transform.png
[image10]: ./Inverse_Kinematics.png

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

Note: The solution provided by Udacity was referenced in my solution.  I had trouble when running the IK.debug.py with offset values around 5 units.  It turned out my calculation of b side was flawed. I used the solution tutorial significantly to debug and correct my errors. 

#### DH Table
![DHTable_Table][image6]

### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.


#### The DH table is stored in a dictionary for quick look up, and used to calculate the individual transfromation matrices.
![DH_Dict_TF_Matrix][image7]


####  The TF_Function as shown below.
![TFFunction][image8]


#### The total transforms from base to gripper is calculated by the product of the consecutive transform matrix.
![TotalTransform][image9]


#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

![alt text][image5]
I had trouble with the inverse kinematics and used the Udacity solution video for reference.
The code below illustrates the solution that provided a very low unit offset and successful completion of the simulator pickup and drop-off.
The side_b presented a major bug in my code.  After correcting my error, the offset was reduced to allow precise placement of the arm.  The Angles of the triangle was solved using the cosine rule.

![Inverse_Kinematics][image10]

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 


The code allowed for sucessfull completion of 2/2 pick and places.  My virtual environment was extremely slow and cumbersome. I'm thankful for the IK_debug.py script for testing.  The main problem error in my code was the calculation of the bside. I was taking the sqrt (x^2 + y^2 - 0.35) and didn't realize. After resolving the bug with help of the Udacity Tutorial my offset was within spec.

There are no plans to modify or work on this code further.  I didn't find this project very enjoyable, and would rather focus on improving the RoverSim project.

Below are snapshots of robot arm simulation doing pickup, movement and drop-off

![pickup][image1]
![Move_1][image2]
![Move_2][image3]
![Drop-off][image4]


