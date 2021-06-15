# ABot

ABot is a differential drive mobile robot based on Robot Operating System (ROS). Its tested on Ubuntu 18.04 with ROS Melodic and Ubuntu 20.04 with ROS Noetic
### Capabilities:

- [x] Manual teleop control.
- [x] SLAM with Gmapping using teleop.
- [x] Goal Point Navigation using DWAPlanner.
- [x] Coverage Planning with Boustrophedon Decomposition.

## Use ABot
To run the ABot simulation in Gazebo, clone the package in your catkin workspace, and use the launch file below.

```console
cd ~/catkin_ws/src
git clone https://github.com/KamalanathanN/abot.git
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## Launch ABot in Gazebo
```console
roslaunch abot_gazebo robot_house_gazebo.launch
```

## ABot Simulation Model 
<img src="data/abot_model_sim.gif" style="zoom:60%;" />

## ABot Gmapping using teleop
```console
roslaunch abot_slam abot_gmapping.launch
roslaunch abot_teleop abot_teleop_key.launch
```
<img src="data/abot_gmapping_house_32x_speed_cropped.gif" style="zoom:80%;" />

## ABot Navigation in Gazebo and Rviz
```console
roslaunch abot_navigation abot_navigation.launch
```
<img src="data/abot_goal_nav.gif" style="zoom:80%;" />

## ABot Path Coverage implementation with Boustrophedon Decomposition
```console
roslaunch abot_path_coverage abot_path_coverage.launch
```
<img src="data/abot_path_coverage_8x_speed_cropped.gif" style="zoom:80%;" />

## References
[path_coverage_ros](https://gitlab.com/Humpelstilzchen/path_coverage_ros/)

## Contributions
Your contributions are most welcomed.
