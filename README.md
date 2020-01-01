# Quadcopter Simulation

Made by Rumaisa Abdulhai

Student at Massachsetts Academy of Math & Science at WPI

## Introduction

This repository uses a quadcopter to map a simple gazebo environment using the gmapping SLAM algorithm and allows the drone to navigate from one point to another using the move_base node. The drone model with a laser sensor has been tested to follow these tasks, but a drone model with a kinect depth camera performing these tasks is a currently a work in progress.

**Note**: A linux machine running ROS Kinetic and Ubuntu 16.04 were used to make this package. This code is not guaranteed to work for other ROS versions, but many packages used in this project are available in other ROS distros.

---
## Installation

This section will cover installing ROS as well as the required packages for the project.

### Getting ROS

Follow the instructions in the link to install the full version of ROS Kinetic and make a catkin_ws folder:
[http://wiki.ros.org/kinetic/Installation/Ubuntu](http://wiki.ros.org/kinetic/Installation/Ubuntu)

### Installing the Packages

Clone the [hector quadrotor](http://wiki.ros.org/hector_quadrotor) stack into your ~/catkin_ws/src folder:

```bash
 # This package is required for the drone model.
 git clone https://github.com/tu-darmstadt-ros-pkg/hector_quadrotor.git
```

Install the [gmapping](http://wiki.ros.org/gmapping) package, [amcl](http://wiki.ros.org/amcl) package, and [move_base](http://wiki.ros.org/move_base) package required for this project:

```bash
 # Packages for mapping, localization, and navigation, respectively
 sudo apt-get install ros-kinetic-gmapping ros-kinetic-amcl ros-kinetic-move-base
```

---

## Running the code for Mapping

In Terminal Tab 1:

```bash
# Runs the Gazebo world and rviz visualization tool
roslaunch quadcopter_gazebo quadcopter.launch
```

In Terminal Tab 2:

```bash
# Runs the gmapping node which creates a map of the environment
roslaunch quadrotor_navigation quadrotor_mapping.launch
```

In Terminal Tab 3:

```bash
# Runs the code for hovering the drone
rosrun takeoff_land takeoff_code.py
```

In Terminal Tab 4:

```bash
# Runs the code for controlling the drone with keyboard
rosrun takeoff_land quad_teleop.py
```

After Mapping in Terminal Tab 5:

```bash
# Saves the map to the desired directory
rosrun map_server mapsaver -f /home/<username>/catkin_ws/src/quadrotor_navigation/maps/new_map
```

After map has been saved, close all terminal tabs.

---

## Running the code for Navigation

In Terminal Tab 1:

```bash
# Runs the Gazebo world and rviz visualization tool
roslaunch quadcopter_gazebo quadcopter.launch
```


In Terminal Tab 2
```bash
# Runs the code for hovering the drone
rosrun takeoff_land takeoff_code.py
```

In Terminal Tab 3
```bash
# Allows 2D nav goal for navigation in Rviz
roslaunch quadrotor_navigation quadrotor_move_base.launch
```

Specify a 2D navigation goal in rviz.

---

## Results

This image shows the drone used in the project:

![The Drone](/images/Drone1.png)

This image shows the environment used in the project:

![The environment](/images/Environment1.png)

This image shows a snapshot of the mapping process:

![The mapping process](/images/MapMiddle.png)

This image shows the resulting map created:

![The map](/images/new_map.png)

This image shows the drone navigating to the goal:

![2D Goal](/images/2DGoal.png)

---

## Demos

Click this [link](https://drive.google.com/open?id=1zvveXdScOxd2hiGnyvaLpDm5zXCi5mQ_) to view the demos for the mapping and navigation.

---

## Recommended Links

I used some of these links to make this package, so feel free to reference them if you want to know more about how the files work together.

ROS Tutorials:

http://wiki.ros.org/ROS/Tutorials

Gazebo Environment:

http://gazebosim.org/tutorials?tut=model_editor

Gazebo/ROS Project with Terrestrial Robot:

http://moorerobots.com/blog

Mapping with Quadcopter and Laser:

https://youtu.be/dND4oCMqmRs

Quadcopter Localization:

https://youtu.be/n6RjVbh3Vgc

Quadcopter Path Planning:

https://youtu.be/JZqVPgu0KIw

---

## Questions

If you have any questions or find any bugs with the code, feel free to email me at rabdulhai@wpi(dot)edu.
