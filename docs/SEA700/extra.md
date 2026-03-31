

### Jetson Nano Board

1. Go through all the Chapter 6 lesson in the [JetAuto & JetAuto Pro Resources](https://drive.google.com/drive/folders/16pwHYO8rK-22oAzStc7-olP9Weq7AbzY) to understand more functionality of the Jetson Nano board.

roslaunch jetauto_peripherals imu.launch
rostopic echo /imu_data

roslaunch jetauto_controller odom_publish.launch
rostopic echo /odom_raw



<!--

1. install ubuntu
2. add jetauto to sudoer
3. install ros
2. copy jetauto_ws
3. source and change permission of devel/setup.py

rosdep install --from-paths src --ignore-src -r -y

sudo apt-get install ros-melodic-slam-gmapping

roslaunch jetauto_gazebo room_worlds.launch
roslaunch jetauto_slam slam.launch sim:=true
roslunach jetsuto_slam rviz_slam.launch sim:=true

-->