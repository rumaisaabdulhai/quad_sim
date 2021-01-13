---
title: Installation
has_children: false
nav_order: 2
---

# Installation
{: .no_toc }

This section will cover installing ROS as well as the required packages for the project.
{: .fs-6 .fw-300 }

<details open markdown="block">
  <summary>
    Table of contents
  </summary>
  {: .text-delta }
1. TOC
{:toc}
</details>

---

## Getting ROS

Follow the instructions in one of the links to install the full version of ROS [Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) 
or [Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu) and make a catkin_ws folder.

## Installing the Packages

Clone the [hector_quadrotor](http://wiki.ros.org/hector_quadrotor), [hector_localization](http://wiki.ros.org/hector_localization), [hector_models](http://wiki.ros.org/hector_models), and [hector_gazebo](http://wiki.ros.org/hector_gazebo) stacks into your ~/catkin_ws/src folder:

```bash
 # Required for the drone model.
 git clone https://github.com/tu-darmstadt-ros-pkg/hector_quadrotor.git
 git clone https://github.com/tu-darmstadt-ros-pkg/hector_localization.git
 git clone https://github.com/tu-darmstadt-ros-pkg/hector_models.git
 git clone https://github.com/tu-darmstadt-ros-pkg/hector_gazebo.git
```

Clone the [rrt_exploration](http://wiki.ros.org/hector_gazebo) package into your ~/catkin_ws/src folder:

```bash
# Used for frontier exploration
 git clone https://github.com/hasauino/rrt_exploration.git
```

Install the [gmapping](http://wiki.ros.org/gmapping), [amcl](http://wiki.ros.org/amcl), and [move_base](http://wiki.ros.org/move_base) packages required for this project:

```bash
 # Packages for mapping, localization, and navigation, respectively
 sudo apt-get install ros-${ROS_DISTRO}-gmapping ros-${ROS_DISTRO}-amcl ros-${ROS_DISTRO}-move-base
```

Follow these [instructions](https://google-cartographer-ros.readthedocs.io/en/latest/compilation.html) to get google cartographer (ROS Melodic only). Do not forget to source your terminal if using cartographer: 

```bash
 # Run command in every terminal you want to use cartographer in
 source install_isolated/setup.bash
```

You may also need to install these packages (ROS Melodic):

```bash
sudo apt-get install ros-melodic-map-server
sudo apt-get install ros-indigo-dwa-local-planner
```

---

## Getting the Code

Clone this [quad_sim](https://github.com/rumaisaabdulhai/quad_sim) package into your ~/catkin_ws/src folder:

```bash
 # This package contains all the code required for mapping and navigation of the drone.
 git clone https://github.com/rumaisaabdulhai/quad_sim.git
```

Make sure the catkin_ws builds successfully (can also use catkin_make):
```bash
 cd ~/catkin_ws
 catkin build
```

Don't forget to source your terminal now, and every time you open a new terminal tab:
```bash
 source ~/catkin_ws/devel/setup.bash
```