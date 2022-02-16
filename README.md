# ABot

ABot is a differential drive mobile robot based on Robot Operating System (ROS). Its tested on Ubuntu 18.04 with ROS Melodic and Ubuntu 20.04 with ROS Noetic

### Capabilities:

- [x] Manual teleop control.
- [x] SLAM with Gmapping using teleop and explore_lite.
- [x] Goal Point Navigation using DWAPlanner.
- [x] Coverage Planning with Boustrophedon Decomposition.
- [x] Configured Behavior Tree 
- [x] Added Intel Realsense D435
- [ ] RTAB-Map

## Use ABot

To run the ABot simulation in Gazebo, clone the package in your catkin workspace, and use the launch file below.

```bash
    cd ~/catkin_ws/src
    git clone https://github.com/KamalanathanN/abot.git
    cd ~/catkin_ws
    catkin_make
    source devel/setup.bash
```

## Launch ABot in Gazebo

```bash
    roslaunch abot_gazebo robot_house_gazebo.launch
```

## ABot Simulation Model 

<img src="data/abot_model_depth.gif" style="zoom:130%;" />

## ABot Gmapping using teleop

```bash
    roslaunch abot_slam abot_gmapping.launch
    roslaunch abot_teleop abot_teleop_key.launch
```
<img src="data/abot_gmapping_house_32x_speed_cropped.gif" style="zoom:80%;" />

## ABot Gmapping with [explore_lite](http://wiki.ros.org/explore_lite)

```
    sudo apt install ros-noetic-explore-lite
```

```bash 
    roslaunch abot_slam abot_gmapping.launch
    roslaunch abot_navigation move_base.launch
    roslaunch explore_lite explore.launch
```
<img src="data/abot_exp.gif" style="zoom:80%;" />

## ABot Navigation in Gazebo and Rviz

```bash
    roslaunch abot_navigation abot_navigation.launch
```
<img src="data/abot_goal_nav.gif" style="zoom:80%;" />

## ABot Path Coverage implementation with Boustrophedon Decomposition

```bash
    roslaunch abot_path_coverage abot_path_coverage.launch
```
<img src="data/abot_path_coverage_8x_speed_cropped.gif" style="zoom:80%;" />

## Abot Behavior Tree Demo

Below is a simple goal points looping behaviour tree with an interrupt action to go to charge port location.

- In future, interrupt action can be used by battery monitoring node to perform auto charge behavior.
- Also planning to include camera feedback actions like object detection for other exiting behaviors :)

### Runtime Behahivor Tree monitoring with Groot

https://user-images.githubusercontent.com/31615486/152803981-b7dd6d3e-62fd-4a26-9e99-6f240f8b9eb8.mp4

Run the below commands in seperate terminals after launching ```robot_house_gazebo.launch``` and ```abot_navigation.launch``` 
```bash
    rosrun groot Groot # select Monitor option and Start

    roslaunch abot_behavior_tree goal_points_looping.launch
```
Notice the STATE of each node in tree gets highlighted with Cyan(IDLE) Yellow(RUNNING) Green(SUCCESS) Red(FAILURE)

### To provide interrput 

```bash
    rostopic pub /interrupt_event std_msgs/String "data: 'Charge_port'"
```

## References

[path_coverage_ros](https://gitlab.com/Humpelstilzchen/path_coverage_ros/)

[Groot with ROS](https://medium.com/teamarimac/groot-with-ros-a8f7855f8e35)

## Contributions

Your contributions are most welcomed.
