#! /usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
 
def takeoff_callback(msg): 
  z_pos = msg.pose.position.z # the current z position of the drone

  # print("X VELOCITY     ", twist_msg.linear.x)
  # print("Y VELOCITY     ", twist_msg.linear.y)

  # if the current x and y velocities are 0
  if twist_msg.linear.x == 0 and twist_msg.linear.y == 0:
    # if the z_pos is between 1 and 1.3 meters, set a small z vel
    if z_pos > 1 and z_pos < 1.3:

      print ("                  SLIGHTLY LOW                  ")
      print
      twist_msg.linear.z = 0.1

    # if the z_pos is between 1.3 and 1.5 meters, hover
    elif z_pos > 1.3 and z_pos < 1.5:
      print ("                    HOVERING                    ")
      twist_msg.linear.z = 0.0
    
    # if the z_pos is between 1.5 and 1.8 meters, set a small neg z vel
    elif z_pos > 1.5 and z_pos < 1.8:
      print ("                    SLIGHTLY HIGH                    ")
      twist_msg.linear.z = -0.1

    # if the z_pos is greater than 1.8 meters, set a med neg z vel
    elif z_pos > 1.8:
      print ("                    OVERSHOT                    ")
      twist_msg.linear.z = -0.3

    # if the drone is below 1 meter, takeoff with pos z vel
    else:
      print ("                    TAKING OFF                    ")
      twist_msg.linear.z = 0.3
  
    # publish z velocity
    cmd_vel_pub.publish(twist_msg)

  # elif twist_msg.linear.x != 0 or twist_msg.linear.y != 0:
  #   print("          NAVIGATING               ")
  #   pass

rospy.init_node('takeoff_land')
rate = rospy.Rate(5)

takeoff_sub = rospy.Subscriber('/ground_truth_to_tf/pose', PoseStamped, takeoff_callback)
cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

twist_msg = Twist()
rospy.spin()