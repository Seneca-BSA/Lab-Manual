# Project 1 - Setting up ROS1 and TurtleSim

<font size="5">
**Due:** Sunday, Feb 1, 2026 at 11:59PM on Blackboard</br>
**Weight:** 10%
</font>

## Introduction

In this project, you will demonstrate your ability to set up a ROS1 environment and create a simple node to control a robot using TurtleSim.

## Objectives

1. Set up the ROS1 environment.
2. Create a simple ROS node to control a robot.
3. Answer assessment questions.

**NOTE:** This project must be completed individually.

- [Seneca Academic Integrity Policy](https://www.senecapolytechnic.ca/about/policies/academic-integrity-policy.html)
- [Seneca Generative Artificial Intelligence (GenAI) Policy](https://www.senecapolytechnic.ca/about/policies/generative-ai-policy.html)

### Step 1: Set Up Your ROS Environment

1. Install ROS1 as per the [Lab 1](lab1.md) instructions.
2. Create your ROS workspace as per the [Lab 3](lab3.md) instructions.

### Step 2: Create and Run a Simple ROS Node

Use open-source code, AI-generated code, or your own code to create a controller as described below (and in [Lab 3](lab3.md)) to make the robot move in TurtleSim.

Create a new package called `lab3_turtlesim`. You can create a new workspace called `lab3_ws` or use your existing workspace.

The commands to create the packages are given below depending on your preferred programming language. You'll need the `geometry_msgs` dependency to use the `twist` object.

Python (Easy) project:

    catkin_create_pkg lab3_turtlesim rospy geometry_msgs

or C++ (Hard) project:

    catkin_create_pkg lab3_turtlesim roscpp geometry_msgs

Your node should do the following:

- Accept a command line argument specifying the name of the turtle it should control.
    - i.e., running `rosrun lab3_turtlesim turtle_controller turtle1` will start a controller node that controls `turtle1`.
- Use `w`, `a`, `s`, `d` (and `q`, `e`, `c`, and `z`) to control the turtle by publishing velocity control messages on the appropriate topic whenever the user presses those keys on the keyboard, as in the original `turtle_teleop_key`. **The turtle should ONLY move when a key is pressed. When the key is released, the turtle should STOP moving.**
    - Option 1 (Easy, max grade 80%) Single keypress: In addition to just forward/backward and turns, the turtle should move forward and turn left in a circular path if `q` is pressed and similar for `e`, `c`, and `z` in their corresponding directions.
    - Option 2 (Hard) Multiple keypress: If you want to challenge your Python skills, make the controller so it listens to multiple keys. i.e., if `w + a` are pressed, the turtle should move forward and turn left in a circular path. If the keys pressed are contradicting, there should be no movement. You'll need to install an additional Python package to achieve this.

**Hint:** You'll need to use the `Twist` message type in the `geometry_msgs` package.

**Hint:** You can use [teleop_turtle_key.cpp](https://docs.ros.org/en/kinetic/api/turtlesim/html/teleop__turtle__key_8cpp_source.html) as a reference for cpp.

To test, spawn multiple turtles and open multiple instances of your new turtle controller node, each linked to a different turtle.

### Step 3: Assessment Questions

1. What command did you use to create the ROS package?
2. Explain why and how you used ROS messages in your program. i.e., Which message and type?
3. Describe the steps to launch ROS, TurtleSim, and your ROS node simultaneously.
4. How do you verify that your ROS node is publishing messages correctly? i.e., What command?

## Submission

1. A link to your project folder (e.g., on GitHub or Google Drive) containing all the necessary files and code.
2. A video showing your code running in TurtleSim and the robot moving.
3. A text file containing your name and the answers to the assessment questions.

**Note:** If you used any GenAI tool for this project, you must also declare which tool and which portion of the code (i.e., line number or function), and/or paragraph was written by GenAI in the text file.

### Late Submission Penalty

1. A 25% reduction from the full mark will be applied for every 24 hours the submission is late after the deadline.