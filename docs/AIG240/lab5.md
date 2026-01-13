# Lab 5: JetAuto Robot

## Procedures

### JetAuto Robot Inspection

The robot we are using for this course is the JetAuto Pro, assembled in the configuration below:

![Figure 5.2 JetAuto Pro](lab5-jetauto-pro.png)

***Figure 4.2** JetAuto Pro*

1. Before using the JetAuto robot, read the following and inspect your robot according to the:

    - [JetAuto User Manual](JetAuto-User-Manual.pdf)
        - Page 01: Guide to Battery Safety
        - Page 03: JetAuto Pro Standard Kit Packing List
        - Page 04-09: Installation Instructions (except for 1.4 LCD)
            - Check all nuts and bolts to confirm installation and security
        - Page 10-11: Charging and Starting the Robot

    We will NOT be using the smartphone app for controlling the robot.

    **NOTE:** All the cables, nuts, and bolts are already installed. You are to **validate** that they are not missing or loose.

    ### SSH into the JetAuto Robot

    !!! warning
    
        Copy these lab instructions somewhere on your computer, as you'll lose connection to the internet once connected to the robot!

1. By default, the JetAuto is configured to be in Wi-Fi AP mode. Power on the robot and connect to the robot's Wi-Fi (starting with "HW-"). If you are unsure of which Wi-Fi SSID your robot is broadcasting, open the "Tool" application on the robot and look for the AP name in the settings. Do NOT change any of the default settings.

    !!! note "Wi-Fi Password"

        The password for the Wi-Fi connection is: **hiwonder**

    Remember, the Wi-Fi AP from the JetAuto has no access to the internet, so it is normal to see "No Internet" or a similar warning when connecting to it.

1. If your robot is making a high-pitched beeping sound, that means the battery voltage is low. Plugging the robot into its charger should solve the problem. However, if the battery hasn't been charged for a while, you might need to leave the robot off and charge it for 10-15 minutes before powering it on.

1. Once connected, use the terminal (or PuTTY) to SSH into the robot at "192.168.149.1".

        ssh jetauto@192.168.149.1

    !!! note "JetAuto User and Password"
    
        - User is: **jetauto**
        - Password is: **hiwonder**

    #### Option 2: USB connection with the robot

    1. It is also possible to connect to the robot via USB using the Jetson Nano's micro-B USB port if you do not want to lose your internet connection. However, you will only be able to use the command line interface with this method.

    1. Use `screen` or a similar serial terminal application to connect with the robot.

            sudo apt-get install -y screen
            sudo screen /dev/ttyACM0 115200

        **NOTE:** The port name may vary depending on your USB connection.

    ### Remote Desktop into the JetAuto Robot

1. Disconnect your computer from the JetAuto robot and re-connect to the internet to [Download and Install NoMachine](https://downloads.nomachine.com/everybody/).

    !!! warning "Only install NoMachine within your virtual machine"
    
        NoMachine is a tool that allows for remote desktop and access. It is a powerful tool that is pre-installed on the JetAuto image, but use it with caution as it will also automatically create and start a remote access server after installation and open up your machine for remote connection.

        For security reasons, do **NOT** install NoMachine on your host computer, and stop the NoMachine server after your installation.

1. After installing NoMachine, reconnect to JetAuto's Wi-Fi AP, and you should be able to search for the JetAuto robot from NoMachine.
    
    Once you are connected to the robot, the credentials are the same as the SSH login above.

1. Now that you have two methods for connecting to the JetAuto robot, you may use either one to control the robot. Keep in mind that the SSH method runs off your virtual machine and connects to JetAuto using the command line, whereas NoMachine connects to the JetAuto robot's actual desktop (remote desktop). The NoMachine GUI requires more connection bandwidth, but tools such as RViz will not work on the command line interface.

1. To conserve battery life, once you've connected to your robot, you may unplug the LCD screen. However, remember that the LCD screen must be plugged in during power ON in order for the robot to detect it and display the desktop.

    !!! info "The LCD screen must be plugged in during power ON in order for the robot to detect it for displaying the desktop!"

    ### JetAuto Robot Movement

    !!! danger
    
        1. Ensure the battery charging cable is UNPLUGGED and all cables on the robot are secure.
        2. Ensure all structures, nuts, and bolts on the robot are tightly fastened.
        3. Ensure the robot is on the ground and away from any obstacles.

1. In a terminal that's connected to the JetAuto robot using SSH, or in JetAuto's terminal using remote desktop, stop the app service and then start the `jetauto_controller` service:

    Stop the app service for Android or iOS connection:

        sudo systemctl stop start_app_node.service

    !!! info

        You must stop this service every time you want to control the robot using your own script through ROS because the JetAuto robot automatically starts an App service to allow for control using an Android or iOS application.

    Launch the robot controller for controlling the robot's hardware.

        roslaunch jetauto_controller jetauto_controller.launch

1. After the controller service has started, we can start publishing move commands as Twist messages to the motion controller (similar to what we did in Lab 4).

    !!! danger "Place the robot on the ground"

        Place the robot on the ground to prevent damage and injury.

    !!! warning "You must issue a stop command for the robot to stop"

        Before publishing a command to the robot, remember you must issue a stop command for the robot to stop.

    Let's issue the stop command first so you can recall it faster.

        rostopic pub -1 /jetauto_controller/cmd_vel geometry_msgs/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'

    Your robot should not do anything. Issue the stop command again by pressing the up arrow key to view the previous command, then hit Enter.

1. Place the robot on the ground and be ready to issue the stop command immediately after issuing the move command that moves the robot in the x-direction (forward) at 0.3 m/s, `x: 0.3`:

        rostopic pub -1 /jetauto_controller/cmd_vel geometry_msgs/Twist '{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'

    Your robot should now start moving. Stop the robot by issuing (or pressing the up arrow twice):

        rostopic pub -1 /jetauto_controller/cmd_vel geometry_msgs/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'

    !!! danger "DO NOT set the linear movement value above 0.7 and angular movement value above 3.5"

        DO NOT set the movement value above 0.7 m/s and angular movement value above 1.0 rad/s to keep the robot within its control limit.

    The linear values refer to the translation speed of the robot. Positive X is forward and positive Y is left. There is no Z-direction for this robot. Do not exceed 0.7 m/s.

    The angular values refer to the rotation speed of the robot. Only Z-rotation is considered, with positive values as counter-clockwise. Do not exceed 1.0 rad/s.

1. Next, try using the controller you created in Lab 4 to control the robot by copying your Lab 4 workspace (`lab4_jetauto_control`) over to the home directory on the JetAuto robot. You do NOT need to copy the `jetauto_ws` workspace as it's already on the JetAuto robot. You may use SSH, SFTP, WinSCP, or any file transfer method.

1. Once your new workspace is on the JetAuto robot, source it the same way as you've done in Lab 3 and Lab 4. Afterward, you'll be able to run the script just like you ran it on Gazebo.

## Lab Question

1. Using the code from Project 2, modify it so the JetAuto robot will move in a real-world roughly 1-meter square pattern as shown below. Remember, you'll be implementing it on the physical JetAuto robot, so the dynamics will not be the same (i.e., your robot will under- or overshoot the movement).

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

For example: when you run `rosrun lab4_jetauto_control jetauto_control`, it should ask for an input before performing the above action.

**Hint:** You can follow the same approach as Lab 3 by creating a new package called `lab4_jetauto_control` in your `ros_ws`.

    catkin_create_pkg lab4_jetauto_control rospy geometry_msgs

Refer to the `teleop_key_control.py` controller you used in [Lab 4](lab4.md) for instructions on how to publish to the JetAuto nodes.

## Reference

- [ROS Tutorials](https://wiki.ros.org/ROS/Tutorials)
- [JetAuto User Manual](JetAuto-User-Manual.pdf)
- EECS 106A Labs