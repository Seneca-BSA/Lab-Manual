# Project 3 - Moving JetAuto in Real World

<font size="5">
**Due:** Sunday, Mar 15, 2026 at 11:59 PM on Blackboard</br>
**Weight:** 10%
</font>

## Introduction

In this project, you will demonstrate your achievement in programming a real robot to navigate a fixed pattern. This project will help you
understand how to communicate and program a real robot.

## Objectives

1. Set up the physical robot.
2. Create a simple ROS node to control a robot in real world.
3. Answer the assessment questions.

**NOTE:** This project must be completed individually.

- [Seneca Academic Integrity Policy](https://www.senecapolytechnic.ca/about/policies/academic-integrity-policy.html)
- [Seneca Generative Artificial Intelligence (GenAI) Policy](https://www.senecapolytechnic.ca/about/policies/generative-ai-policy.html)

### Step 1: Set Up and Inspect JetAuto Robot

1. In groups of up to 3 students, set up and inspect the JetAuto Robot.
2. Follow the instructions provided by the instructor.

### Step 2: Move the JetAuto robot

Use open-source code, AI-generated code, or your own code to create a control script as described below to make the physical JetAuto robot move according to the same pattern from Lab 4 (Project 2).

Write code that will move the JetAuto robot in a roughly 1-meter square shape pattern as follows:

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

Run this on the real world robot.

## Assessment Questions

1. How did you establish communication with the robot?
2. What are the difference in command on controlling the robot in Gazebo vs the real robot?
3. What are the difference in the control value between moving the robot roughly 1 meter in Gazebo vs the real robot?
4. What challenges did you face when getting the real robot to move?

## Submission

1. A link to your project folder (e.g., on GitHub (private) or Google Drive) containing all the necessary files and code.
2. A video showing your robot running in the real world.
3. A text file containing the answers to the questions for every group member.

### Late Submission Penalty

1. A 25% reduction from the full mark will be applied for every 24 hours the submission is late after the deadline.