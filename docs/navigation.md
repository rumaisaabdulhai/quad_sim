---
layout: default
title: Navigation
nav_order: 4
has_children: false
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
