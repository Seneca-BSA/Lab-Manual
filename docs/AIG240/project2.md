# Project 2 - Moving JetAuto in Gazebo

<font size="5">
**Due:** Sunday, Feb 15, 2026, at 11:59 PM on Blackboard</br>
**Weight:** 10%
</font>

## Introduction

In this project, you will demonstrate your proficiency in setting up a ROS1 environment and creating a simple node to control a robot using Gazebo.

## Objectives

1. Set up the Gazebo environment to work with ROS.
2. Create a simple ROS node to control a robot in Gazebo.
3. Answer assessment questions.

**NOTE:** This project must be completed individually.

- [Seneca Academic Integrity Policy](https://www.senecapolytechnic.ca/about/policies/academic-integrity-policy.html)
- [Seneca Generative Artificial Intelligence (GenAI) Policy](https://www.senecapolytechnic.ca/about/policies/generative-ai-policy.html)

### Step 1: Set Up Your Gazebo Environment

1. Install Gazebo as per the [Lab 4](lab4.md) instructions.
2. Create your new ROS workspace (or use the same one) as per the [Lab 3](lab3.md) instructions.

### Step 2: Create and Run a Simple ROS Node

Use open-source code, AI-generated code, or your own code to create a control script as described below to make the JetAuto robot move according to a pattern in Gazebo.

Write a code that will move the JetAuto robot in a roughly 1-meter square pattern as follows:

![Figure 1 Square Movement Pattern](lab4-task.png)

***Figure 1** Square Movement Pattern*

Start:

1. Move forward from (0, 0, 0°) to (1, 0, 0°) facing the direction of travel; then
2. Move sideways to the left from (1, 0, 0°) to (1, 1, 0°) without turning, so the robot is facing the outside of the square; then
3. Turn clockwise from (1, 1, 0°) to (1, 1, -90°) to face into the square; then
4. Move sideways to the right from (1, 1, -90°) to (0, 1, -90°) facing the inside of the square; then
5. Move forward and turn from (0, 1, -90°) to (0, 0, 0°) by rotating the robot while traveling.
    1. (Easy option, max grade 80%) Move the robot forward, then turn once it reaches the start point.

Repeat this twice after a start command (such as a keyboard input) is given.

For example: after launching the JetAuto in Gazebo, when you run `rosrun lab4_jetauto_control jetauto_control`, it should ask for an input before performing the above action.

**Hint:** You can follow the same approach as Lab 3 by creating a new package called `lab4_jetauto_control` in your `ros_ws`.

    catkin_create_pkg lab4_jetauto_control rospy geometry_msgs

Refer to the `teleop_key_control.py` controller you used in [Lab 4](lab4.md) for instructions on how to publish to the JetAuto nodes.

## Assessment Questions

1. What command did you use to launch the JetAuto robot in Gazebo?
2. Which two other launch files were called when you launched `worlds.launch`? **Hint:** Do not use GenAI to answer this because it does not have access to the file. Inspect the launch files in `jetauto_ws/src/jetauto_simulation/jetauto_gazebo/launch`.
3. Describe the process of setting up your ROS workspace and creating the project package.
4. (Answer this only if you completed the advanced turning option) What challenges did you face with making the robot move in a straight line while turning, and how did you overcome them?

## Submission

1. A link to your project folder (e.g., on GitHub (private) or Google Drive) containing all the necessary files and code.
2. A video showing your code running in Gazebo and the robot moving.
3. A text file containing your name and the answers to the assessment questions.

**Note:** If you used any GenAI tool for this project, you must also declare within the text file which tool was used and which portion of the code (i.e., line number or function) and/or paragraph was written by GenAI.

### Late Submission Penalty

1. A 25% reduction from the full mark will be applied for every 24 hours the submission is late after the deadline.