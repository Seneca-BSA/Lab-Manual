# Project 5 - JetAuto Navigation

<font size="5">
**Due:** Sunday, Apr 12, 2026 at 11:59 PM on Blackboard<br />
**Weight:** 20%
</font>

## Introduction

In this project, you will build on the map you created in Project 4 (or a map provided to you) and run autonomous navigation using the available navigation package in the robot. You will use the ROS navigation packages to allow the robot to autonomously move within the mapped environment. The robot will use the previously generated map to localize itself and plan paths to target locations. You will interact with the navigation system using RViz to set navigation goals and monitor the
robot’s behavior.

## Objectives

1. Understand the concept of autonomous navigation.
1. Learn how robots use maps to localize themselves in an environment.
1. Understand path planning and motion planning concepts.
1. Learn how robots avoid obstacles using onboard sensors.
1. Analyze navigation performance and system behavior.

## Project Tasks

1. In groups of up to 3 students, set up the JetAuto robot for autonomous navigation per [Lab 7](lab7.md).

### Scenario 1: Navigating without obstacles

1. Load the saved map from Project 4 (or a map provided to you)
1. Launch the JetAuto ROS navigation stack.
1. Initialize the robot's localization system (using RViz or script).
1. Set a series of target poses (goal position and orientation), or "waypoint", using a script.
    - if you are using your own map, the 4 waypoints must be at least 3 m apart and have no direct path between them.
1. Observe (and record) the robot navigating autonomously to the goal location.

### Scenario 2: Navigating with obstacles

1. Place some obstacles within the mapped environment.
1. Perform the same script from Scenario 1.
1. Observe (and record) how the robot plans a path around obstacles.
1. Monitor the robot’s local and global planning behavior.
1. Note how the robot reacts to obstacles and changes in the environment.

## Assessment Questions

1. What is the difference between mapping and navigation in robotics?
1. What is localization and why is it necessary for navigation?
1. What is the role of the global planner in the navigation stack?
1. What is the role of the local planner?
1. How does the robot detect and avoid obstacles?
1. What happens if the robot cannot find a valid path to the goal?
1. How does RViz help during the navigation process?

## Submission

1. A video and screen recording showing the robot completing Scenario 1.
1. A video and screen recording showing the robot completing Scenario 2.
1. The map files (map.pgm and map.yaml) that your robot uses for navigation.
1. A link to your project folder (e.g., on GitHub (private) or Google Drive) containing all the necessary files and code.
1. A text file containing the answers to the questions (one per group).

### Late Submission Penalty

1. A 25% reduction from the full mark will be applied for every 24 hours the submission is late after the deadline.