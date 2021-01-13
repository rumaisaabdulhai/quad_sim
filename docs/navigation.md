---
title: Navigation
has_children: false
nav_order: 4
---

# Navigation
{: .no_toc }

This section will go over the navigation methods possible with the quadcopter.
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

### Demos

Here is a demo using move_base and amcl to allow a quadcopter to autonomously navigate in a simple indoor environment. Note: the instructions in this video are outdated, but the process is the same.
<iframe width="560" height="315" src="https://www.youtube.com/embed/QdkYYYw5Tec" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

Here is a demo using move_base and amcl to allow a quadcopter to autonomously navigate in a complex indoor environment.
<iframe width="560" height="315" src="https://www.youtube.com/embed/cq0wKcm7asA" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

## Behind The Code

For the navigation phase, the [amcl](http://wiki.ros.org/amcl) and [move_base](http://wiki.ros.org/move_base) nodes are mainly used. AMCL stands for Adaptive Monte-Carlo Localization. It is a localization method which uses a particle filter to estimate the pose (position and orientation) of the drone in the environment. The node subscribes to the current laser scans and the occupancy grid map which are compared against each other to localize the drone. The node outputs or publishes the particle filter which contains all the estimates of the robot's pose.

The move_base node allows for the 2D navigation goal to be used in rviz. It listens for the goal to be set in rviz and outputs the necessary velocities needed for the drone to navigate from its current pose to the goal pose. The node contains the global and local path planners. The global planner makes a path from the start point to the goal point. Some global path planning methods include A*, RRT*, and Dijikstra's Algorithm, but this code uses Dijikstra's Algorithm. You can change the global planner used in the yaml parameter files. The local path planning method used is called the [dwa_local_planner](http://wiki.ros.org/dwa_local_planner) (Dynamic Window Approach). For each time step, the local planner generates sample velocities that the robot can take. It predicts the possible trajectory of the robot with each of the sample velocities and finds the optimal trajectory based on the distance to the goal, speed of the robot, distance to obstacles, and the robot's distance to the global path.

Both nodes are required for the drone to successfully navigate from the source point to the destination point. You can use rviz to view the local path planning trajectory or the particle filters in real time.