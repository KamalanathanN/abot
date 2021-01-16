# ABot

ABot is a differential drive mobile robot based on Robot Operating System (ROS). Its working on Ubuntu 18.04 with ROS Melodic.
### Capabilities:

- [x] Manual teleop control.
- [x] SLAM with Gmapping using teleop.
- [x] Goal Point Navigation using DWAPlanner.

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
<img src="data/abot_gmapping_house_32x_speed.gif" style="zoom:80%;" />

## ABot Navigation in Gazebo and Rviz
```console
roslaunch abot_navigation abot_navigation.launch
```
## Contributions
Your contributions are most welcomed.
