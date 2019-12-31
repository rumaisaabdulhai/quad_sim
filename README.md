# Running the code for Mapping

Terminal Tab 1
roslaunch quadcopter_gazebo quadcopter.launch

Terminal Tab 2
roslaunch quadrotor_navigation quadrotor_mapping.launch

Terminal Tab 3
rosrun takeoff_land takeoff_code.py

Terminal Tab 4
rosrun takeoff_land quad_teleop.py

After Mapping in Terminal 5
rosrun map_server mapsaver -f /home/<username>/catkin_ws/src/quadrotor_navigation/maps/new_map

After map has been saved, close all terminal tabs

# Running the code for Navigation

Terminal Tab 1
roslaunch quadcopter_gazebo quadcopter.launch

Terminal Tab 2
rosrun takeoff_land takeoff_code.py

Terminal Tab 3
roslaunch quadrotor_navigation quadrotor_move_base.launch

Specify a 2D navigation goal in rviz
