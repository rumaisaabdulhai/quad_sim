# Autonomous Indoor Drone Application

Developed by Rumaisa Abdulhai (Aug 2019 - Present)

Student at Massachsetts Academy of Math & Science at WPI

## Motivation

In the summer of 2019, I attended the MIT Beaver Works (BWSI) UAV Racing Course where I used an Intel RTF research drone to build an autonomous drone application. I gained a practical understanding of the Intel compute board, the Robot Operating System (ROS), the PX4 flight controller, and the sensors, including cameras which allowed the drone to hover with stability at specified heights, identify AR markers on obstacles (for basic obstacle avoidance), and navigate a curved path successfully. My team won first place in the final UAV racing competition, with a time of 1 minute and 32 seconds. BWSI encouraged me to start building my own indoor autonomous drone application. I would like to thank my instructors Ross Allen and Mark Mazumder for motivating me to embark on this project.

## Introduction

This repository allows a quadcopter with a Hokuyo 2D lidar to:

1) Map an indoor gazebo environment using the gmapping SLAM algorithm and teleoperation (ROS Kinetic and Melodic)
2) Map an indoor gazebo environment using the google cartographer SLAM algorithm and teleoperation (ROS Melodic)
4) Map an indoor gazebo environment using the gmapping SLAM algorithm and frontier exploration (ROS Kinetic)
3) Autonomously navigate from one location to another using the move_base node (ROS Kinetic and Melodic)

I am currently working on:

1) Improving the mapping component of this project with frontier exploration. I also hope to integrate frontier exploration with google cartographer.
2) Improving the navigation component of this project by possibly using a depth camera instead of a 2D lidar sensor.
3) Integrating the PX4 flight controller to be able to test this project in real life.

**Note**: There are two versions of this package. The first is compatible with ROS Kinetic and Ubuntu 16.04, and the second is compatible with ROS Melodic and Ubuntu 18.04. The Melodic version is also the master code. I am no longer maintaining the kinetic version, but I am currently working on the melodic version. Please see above for what you can do with each version.

You can view my [thesis](http://users.wpi.edu/~rabdulhai/docs/Thesis.pdf) here.

---

## Table of Contents

1. [Installation](#installation)
    - [Getting ROS](#getting-ros)
    - [Installing the Packages](#installing-the-packages)
    - [Getting the Code](#getting-the-code)
2. [Mapping](#mapping)
    - [Gmapping with Teleop](#gmapping-with-teleop)
    - [Gmapping with Frontier Exploration](#gmapping-with-frontier-exploration)
    - [Google Cartographer with Teleop](#google-cartographer-with-teleop)
3. [Navigation with move_base](#navigation-with-move_base)
4. [Video Streaming](#video-streaming)
5. [Results](#results)
6. [Demos](#demos)
7. [Behind the Code](#behind-the-code)
    - [Mapping Phase](#mapping-phase)
    - [Navigation Phase](#navigation-phase)
8. [Recommended Links](#recommended-links)
9. [Questions](#questions)

---

## Installation

This section will cover installing ROS as well as the required packages for the project.

### Getting ROS

Follow the instructions in one of the links to install the full version of ROS [Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) 
or [Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu) and make a catkin_ws folder.

### Installing the Packages

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

### Getting the Code

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

---

## Mapping

This section will go over the different methods of mapping possible with the quadcopter.

### Gmapping with Teleop

These instructions work with both ROS Kinetic and Melodic.

Make sure `use_ground_truth_for_tf` is set to `true` in the `quadcopter_gazebo/quadcopter.launch` file.

In Terminal Tab 1:

```bash
# Runs the Gazebo world
roslaunch quadcopter_gazebo quadcopter.launch
```

In Terminal Tab 2:

```bash
# Runs the rviz visualization tool and gmapping node which creates a map of the environment
roslaunch quadcopter_mapping quadcopter_gmapping.launch
```

In Terminal Tab 3:

```bash
# Runs the code for hovering the drone
roslaunch quadcopter_takeoff_land quadcopter_takeoff_land.launch
```

In Terminal Tab 4:

```bash
# Runs the code for controlling the drone with keyboard
roslaunch quadcopter_teleop quadcopter_teleop.launch
```

After Mapping in Terminal Tab 5:

```bash
# Saves the map to the desired directory
rosrun map_server map_saver -f /home/<username>/catkin_ws/src/quad_sim/quadcopter_mapping/maps/new_map
```

After map has been saved, land the drone in Terminal Tab 6:

```bash
# Lands the drone
rostopic pub /quadcopter_land -r 5 std_msgs/Empty "{}"
```

Now, you can close all terminal tabs with `CTRL-C`.

---

### Gmapping with Frontier Exploration

At the moment, these instructions will only work for ROS Kinetic. I am getting it to work for ROS Melodic.

Make sure `use_ground_truth_for_tf` is set to `true` in the `quadcopter_gazebo/quadcopter.launch` file.

In Terminal Tab 1:

```bash
# Runs the Gazebo world
roslaunch quadcopter_gazebo quadcopter.launch
```

In Terminal Tab 2:

```bash
# Runs the code for hovering the drone
roslaunch quadcopter_takeoff_land quadcopter_takeoff_land.launch
```

In Terminal Tab 3:

```bash
# Runs the code for frontier exploration, rviz, gmapping, and move_base
roslaunch quadcopter_exploration quadcopter_exploration.launch
```

After Mapping in Terminal Tab 4:

```bash
# Saves the map to the desired directory
rosrun map_server map_saver -f /home/<username>/catkin_ws/src/quad_sim/quadcopter_mapping/maps/new_map
```

After map has been saved, land the drone in Terminal Tab 5:

```bash
# Lands the drone
rostopic pub /quadcopter_land -r 5 std_msgs/Empty "{}"
```

Now, you can close all terminal tabs with `CTRL-C`.

---

### Google Cartographer with Teleop

These instructions only work with ROS Melodic.

Make sure `use_ground_truth_for_tf` is set to `false` in the `quadcopter_gazebo/quadcopter.launch` file.

In Terminal Tab 1:

```bash
# Runs the Gazebo world
roslaunch quadcopter_gazebo quadcopter.launch
```

In Terminal Tab 2:

```bash
# Runs the rviz visualization tool and cartographer node which creates a map of the environment
roslaunch quadcopter_mapping quadcopter_cartographer.launch
```

In Terminal Tab 3:

```bash
# Runs the code for hovering the drone
roslaunch quadcopter_takeoff_land quadcopter_takeoff_land.launch
```

In Terminal Tab 4:

```bash
# Runs the code for controlling the drone with keyboard
roslaunch quadcopter_teleop quadcopter_teleop.launch
```

After Mapping in Terminal Tab 5:

```bash
# Saves the map to the desired directory
rosrun map_server map_saver -f /home/<username>/catkin_ws/src/quad_sim/quadcopter_mapping/maps/new_map
```

After map has been saved, land the drone in Terminal Tab 6:

```bash
# Lands the drone
rostopic pub /quadcopter_land -r 5 std_msgs/Empty "{}"
```

Now, you can close all terminal tabs with `CTRL-C`.

---

## Navigation with move_base

These instructions work with both ROS Kinetic and Melodic.

Make sure `use_ground_truth_for_tf` is set to `true` in the `quadcopter_gazebo/quadcopter.launch` file.

In Terminal Tab 1:

```bash
# Runs the Gazebo world
roslaunch quadcopter_gazebo quadcopter.launch
```

In Terminal Tab 2:
```bash
# Runs the code for hovering the drone
roslaunch quadcopter_takeoff_land quadcopter_takeoff_land.launch
```

In Terminal Tab 3:
```bash
# Runs the rviz visualization tool and allows 2D nav goal for navigation in Rviz
roslaunch quadcopter_navigation quadcopter_move_base.launch
```

Specify a 2D navigation goal in Rviz.

After navigating to the desired goal, land the drone in Terminal Tab 4:

```bash
# Lands the drone
rostopic pub /quadcopter_land -r 5 std_msgs/Empty "{}"
```

Now, you can close all terminal tabs with `CTRL-C`.

---

## Video Streaming

You can view the front camera through Rviz, but you can also use video streaming. Install the [web_video_server](http://wiki.ros.org/web_video_server) package on your laptop:

```bash
# Installs the video streaming package in the opt folder
sudo apt-get install ros-kinetic-web-video-server
```

In order to create the server for the stream, you must be running the Gazebo environment in one Terminal Tab:

```bash
# Runs the Gazebo world
roslaunch quadcopter_gazebo quadcopter.launch
```

In another terminal tab, run the following line to start the server:

```bash
# Starts the sever
rosrun web_video_server web_video_server
```

Now, go to the following [site](http://0.0.0.0:8080/) on your browser. You will see the rostopic for the front camera image. Click on the first link from the left.

You will now see the video stream of the drone from the front camera. You may wish to increase the height and width of the stream or change the type of the stream like so:

http://0.0.0.0:8080/stream_viewer?topic=/front_cam/camera/image&type=mjpeg&width=800&height=600

For more information, visit the following [reference](https://msadowski.github.io/ros-web-tutorial-pt3-web_video_server/).

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

Please visit my [YouTube Channel](https://www.youtube.com/channel/UCfvje9FSd2gTdbsGxQ2Hmqg) to view demonstrations of each component.

---

## Behind the Code

### Mapping Phase

The [gmapping](http://wiki.ros.org/gmapping) node is an implementation of Simultaneous Localization & Mapping (SLAM). The node's input is the laser scan data from the Hokuyo 2D LIDAR sensor and the output is a 2D probabilistic occupancy grid map of the environment. The occupancy grid map continously updates as the drone explores the environment (teleoperated). The white space on the map means free space, the black area means obstacles, and the grey area means the area is unexplored. Each square on the discrete map represents the probability that it is an obstacle.

### Navigation Phase

For the navigation phase, the [amcl](http://wiki.ros.org/amcl) and [move_base](http://wiki.ros.org/move_base) nodes are mainly used. AMCL stands for Adaptive Monte-Carlo Localization. It is a localization method which uses a particle filter to estimate the pose (position and orientation) of the drone in the environment. The node subscribes to the current laser scans and the occupancy grid map which are compared against each other to localize the drone. The node outputs or publishes the particle filter which contains all the estimates of the robot's pose.

The move_base node allows for the 2D navigation goal to be used in rviz. It listens for the goal to be set in rviz and outputs the necessary velocities needed for the drone to navigate from its current pose to the goal pose. The node contains the global and local path planners. The global planner makes a path from the start point to the goal point. Some global path planning methods include A*, RRT*, and Dijikstra's Algorithm, but this code uses Dijikstra's Algorithm. You can change the global planner used in the yaml parameter files. The local path planning method used is called the [dwa_local_planner](http://wiki.ros.org/dwa_local_planner) (Dynamic Window Approach). For each time step, the local planner generates sample velocities that the robot can take. It predicts the possible trajectory of the robot with each of the sample velocities and finds the optimal trajectory based on the distance to the goal, speed of the robot, distance to obstacles, and the robot's distance to the global path.

Both nodes are required for the drone to successfully navigate from the source point to the destination point. You can use rviz to view the local path planning trajectory or the particle filters in real time.

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

You can also email me at [rabdulhai@wpi.edu](mailto:rabdulhai@wpi.edu).
