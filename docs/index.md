---
layout: default
title: Home
nav_order: 1
description: "An autonomous indoor drone application using Gazebo and ROS"
permalink: /
---

# Autonomous Indoor Drone Application
{: .fs-9 }

Developed by Rumaisa Abdulhai (Aug 2019 - Present)<br>
{: .fs-6 .fw-300 }

[Get started now](/quad_sim/installation.html){: .btn .btn-primary .fs-5 .mb-4 .mb-md-0 .mr-2 } [View it on GitHub](https://github.com/rumaisaabdulhai/quad_sim){: .btn .fs-5 .mb-4 .mb-md-0 }

---

## Motivation

In the summer of 2019, I attended the MIT Beaver Works Aerial Vehicle Racing Course where I learned to develop code for an Intel RTF research drone to navigate an obstacle course autonomosly. I gained a practical understanding of the Intel compute board, the Robot Operating System (ROS), the PX4 flight controller, and the sensors, including cameras which allowed the drone to stably hover with stability at specified heights, identify AR markers on obstacles (for basic obstacle avoidance), and navigate a curved path successfully. Beaver Works encouraged me to start building my own indoor autonomous drone application. I would like to thank my instructors for motivating me to embark on this project.

## Introduction

This repository allows a quadcopter with a Hokuyo 2D lidar to:

1. Map an indoor environment using the gmapping SLAM algorithm and teleoperation
2. Map an indoor environment using the google cartographer SLAM algorithm and teleoperation
3. Map an indoor environment using the gmapping SLAM algorithm and frontier exploration
4. Autonomously navigate from one location to another using the move_base node

I am currently working on:

1. Improving the mapping component by integrating frontier exploration with google cartographer.
2. Improving the navigation component by possibly using a depth camera instead of a 2D LiDAR.
3. Integrating the PX4 flight controller to be able to test this project in real life.

**Note**: There are two versions of this package. The first is compatible with ROS Kinetic and Ubuntu 16.04, and the second is compatible with ROS Melodic and Ubuntu 18.04. The Melodic version is also the master code. I am no longer maintaining the kinetic version, but I am currently working on the melodic version.

You can view my [thesis](http://users.wpi.edu/~rabdulhai/docs/Thesis.pdf) here.

---

## Demos

Please visit my [YouTube Channel](https://www.youtube.com/channel/UCfvje9FSd2gTdbsGxQ2Hmqg) to view demonstrations of each component.

1. [Mapping - GMapping, Teleop, Simple Env (Dec 2019)](https://youtu.be/1V5ocwOdLMg)
2. [Navigation - Move Base, Simple Env (Dec 2019)](https://youtu.be/QdkYYYw5Tec)
3. [Massachusetts Science Fair (April 2020 - Overview Video)]()
4. [Mapping - GMapping, Teleop, Complex Env (Mar 2020)](https://youtu.be/-c8N1ncmt2Q)
5. [Navigation w/ live streaming - Move Base, Complex Env (Mar 2020)](https://youtu.be/JbNfKr267cY)
6. [Mapping - GMapping, RRT Exploration, Simple Env (Jul 2020)](https://youtu.be/SNdfzReCWJQ)
7. [Mapping - Google Cartographer, Teleoperation, Complex Env (Nov 2020)](https://youtu.be/KcQ23XDVEuY)
8. [Navigation - Move Base, Complex Env with Cartographer Map (Nov 2020)](https://youtu.be/saH9n_xpQXI)

---

## Recommended Links

I used some of these links to make this package, so feel free to reference them if you want to know more about how the files work together.

1. [ROS Tutorials](http://wiki.ros.org/ROS/Tutorials)
2. [Gazebo Environment](http://gazebosim.org/tutorials?tut=model_editor)
3. [Gazebo/ROS Project with Terrestrial Robot](http://moorerobots.com/blog)
4. [Mapping with Quadcopter and Laser](https://youtu.be/dND4oCMqmRs)
5. [Quadcopter Localization](https://youtu.be/n6RjVbh3Vgc)
6. [Quadcopter Path Planning](https://youtu.be/JZqVPgu0KIw)
7. [Frontier Exploration](http://wiki.ros.org/rrt_exploration/Tutorials)

These links are also helpful (ROS Melodic):

- [REST request issue](https://github.com/ros-industrial/universal_robot/issues/412)
- [canTransform issue](https://github.com/ros-planning/navigation/issues/794)

---

## Questions

If you have any questions or find any bugs with the code, feel free to create an issue in this repository with a screenshot of the error you have. Please be as specific as you can with details, such as what OS or version of ROS you are using and what steps you have taken so far.

You can also email me at [rumaisa@mit.edu](mailto:rumaisa@mit.edu).