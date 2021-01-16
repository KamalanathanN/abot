# ABot

ABot is a differential drive mobile robot based on Robot Operating System (ROS). Its working on Ubuntu 18.04 with ROS Melodic.

# To Use ABot
To run the ABot simulation in Gazebo, clone the package in your catkin workspace, and use the launch file below.

```console
cd ~/catkin_ws/src
git clone https://github.com/KamalanathanN/abot.git
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

# To Launch ABot in Gazebo
```console
roslaunch abot_gazebo robot_house_gazebo.launch
```

## ABot Simulation Model 
<img src=/>

## ABot Navigation Video Links
```console
roslaunch abot_navigation abot_navigation.launch
```
## Contributions
Your contributions are most welcomed.