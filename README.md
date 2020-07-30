# Quadcopter Simulation

Made by Rumaisa Abdulhai

Student at Massachsetts Academy of Math & Science at WPI

## Motivation

In the summer of 2019, I attended the MIT Beaverworks (BWSI) UAV Racing Course where I used an Intel RTF research drone to build an autonomous drone application. I gained a practical understanding of the Intel compute board, the Robot Operating System (ROS), the PX4 flight controller, and the sensors, including cameras which allowed the drone to hover with stability at specified heights, identify AR markers on obstacles (for basic obstacle avoidance), and navigate a curved path successfully. My team won first place in the final UAV racing competition, with a time of 1 minute and 32 seconds. The BWSI program encouraged me to start building my own indoor autonomous drone application. I would like to thank my instructors Ross Allen and Mark Mazumder for motivating me to embark on this project.

## Introduction

This repository uses a quadcopter to map a simple gazebo environment using the gmapping SLAM algorithm and allows the drone to autonomously navigate from one point to another using the move_base node. The drone model with a laser sensor has been tested to follow these tasks, but a drone model with a kinect depth camera performing these tasks is a currently a work in progress.

**Note**: A linux machine running ROS Kinetic and Ubuntu 16.04 were used to make this package. This code is not guaranteed to work for other ROS versions, but many packages used in this project are available in other ROS distros.

---

## Installation

This section will cover installing ROS as well as the required packages for the project.

### Getting ROS

Follow the instructions in the link to install the full version of ROS Kinetic and make a catkin_ws folder:
[http://wiki.ros.org/kinetic/Installation/Ubuntu](http://wiki.ros.org/kinetic/Installation/Ubuntu)

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

Run this [script](https://gist.github.com/kdaun/51b7d19bbcd5e0798c8415cd093078fb) in your ~/catkin_ws folder for installing [Cartographer](https://google-cartographer-ros.readthedocs.io/en/latest/index.html).

Install the [gmapping](http://wiki.ros.org/gmapping), [amcl](http://wiki.ros.org/amcl), and [move_base](http://wiki.ros.org/move_base) packages required for this project:

```bash
 # Packages for mapping, localization, and navigation, respectively
 sudo apt-get install ros-kinetic-gmapping ros-kinetic-amcl ros-kinetic-move-base
```

---

## Getting the Code

Clone this [quad_sim](https://github.com/rumaisaabdulhai/quad_sim) package into your ~/catkin_ws/src folder:

```bash
 # This package contains all the code required for mapping and navigation of the drone.
 git clone https://github.com/rumaisaabdulhai/quad_sim.git
```

Make sure the catkin_ws builds successfully: (can also use catkin_make)
```bash
 cd ~/catkin_ws
 catkin build
```

Don't forget to source your terminal now, and every time you open a new terminal tab:
```bash
 source ~/catkin_ws/devel/setup.bash
```

If running cartographer, source this as well when in the ~/catkin_ws directory:
```bash
 source install_isolated/setup.bash
```

---

## Running the code for Exploration

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
rosrun map_server map_saver -f /home/<username>/catkin_ws/src/quad_sim/quadcopter_navigation/maps/new_map
```

After map has been saved, land the drone in Terminal Tab 5:

```bash
# Lands the drone
rostopic pub /quadcopter_land -r 5 std_msgs/Empty "{}"
```

Now, you can close all terminal tabs with `CTRL-C`.

---

## Running the code for Mapping

In Terminal Tab 1:

```bash
# Runs the Gazebo world
roslaunch quadcopter_gazebo quadcopter.launch
```

In Terminal Tab 2:

```bash
# Runs the rviz visualization tool and gmapping node which creates a map of the environment
roslaunch quadcopter_mapping quadcopter_mapping.launch
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
rosrun map_server map_saver -f /home/<username>/catkin_ws/src/quad_sim/quadcopter_navigation/maps/new_map
```

After map has been saved, land the drone in Terminal Tab 6:

```bash
# Lands the drone
rostopic pub /quadcopter_land -r 5 std_msgs/Empty "{}"
```

Now, you can close all terminal tabs with `CTRL-C`.

---

## Running the code for Navigation

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

Click this [link](https://drive.google.com/open?id=1zvveXdScOxd2hiGnyvaLpDm5zXCi5mQ_) to view the demos for the mapping and navigation. Each video is about 3 minutes long.

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

Frontier Exploration:

http://wiki.ros.org/rrt_exploration/Tutorials

---

## Questions

If you have any questions or find any bugs with the code, feel free to email me at rabdulhai@wpi(dot)edu.
