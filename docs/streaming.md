---
title: Video Streaming
has_children: false
nav_order: 5
---

# Video Streaming

This section will go over how to view the live video from the quadcopter on a web server.
{: .fs-6 .fw-300 }

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

## Demos

Here is a demo of the navigation process with livestreaming.
<iframe width="560" height="315" src="https://www.youtube.com/embed/JbNfKr267cY" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

For more information, visit the following [reference](https://msadowski.github.io/ros-web-tutorial-pt3-web_video_server/).