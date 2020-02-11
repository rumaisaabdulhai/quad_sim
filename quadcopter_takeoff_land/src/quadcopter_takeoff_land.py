#! /usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Empty

import actionlib
from move_base_msgs.msg import MoveBaseActionFeedback

class TakeoffLand():

  def __init__(self):

    rospy.init_node('takeoff_land')

    self.landing = False
    self.isGoal = False
    self.twist_msg = Twist()
    self.twist_active = True

    self.land_sub = rospy.Subscriber('/move_base/feedback', MoveBaseActionFeedback, self.goal_callback)
    self.cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
    self.takeoff_sub = rospy.Subscriber('/ground_truth_to_tf/pose', PoseStamped, self.takeoff_callback)
    self.land_sub = rospy.Subscriber('/quadcopter_land', Empty, self.land_callback)
    self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
   
    self.rate = rospy.Rate(5)
    rospy.spin()

  def takeoff_callback(self,msg):

    z_pos = msg.pose.position.z # the current z position of the drone

    if self.landing == False and self.isGoal == False:

      # if the current x and y velocities are 0
      if self.twist_msg.linear.x == 0 and self.twist_msg.linear.y == 0:
        # if the z_pos is between 1 and 1.3 meters, set a small z vel
        if z_pos > 1 and z_pos < 1.3:

          print ("                  SLIGHTLY LOW                  ")
          self.twist_msg.linear.z = 0.1

        # if the z_pos is between 1.3 and 1.5 meters, hover
        elif z_pos > 1.3 and z_pos < 1.5:
          print ("                    HOVERING                    ")
          self.twist_msg.linear.z = 0.0
        
        # if the z_pos is between 1.5 and 1.8 meters, set a small neg z vel
        elif z_pos > 1.5 and z_pos < 1.8:
          print ("                    SLIGHTLY HIGH                    ")
          self.twist_msg.linear.z = -0.1

        # if the z_pos is greater than 1.8 meters, set a med neg z vel
        elif z_pos > 1.8:
          print ("                    OVERSHOT                    ")
          self.twist_msg.linear.z = -0.3

        # if the drone is below 1 meter, takeoff with pos z vel
        else:
          print ("                    TAKING OFF                    ")
          self.twist_msg.linear.z = 0.3
      
        # publish z velocity
        self.cmd_vel_pub.publish(self.twist_msg)

    elif self.isGoal == True:
      if self.twist_active == False:
        self.isGoal = False
      else:
        print ("                    THERE IS A GOAL                    ")

    elif self.landing == True:

      if z_pos >= 0.3:
        self.twist_msg.linear.z = -0.3
        self.cmd_vel_pub.publish(self.twist_msg)
        print ("                    LANDING                    ")
      else:
        rospy.loginfo("System is shutting down. Stopping drone...")
        rospy.signal_shutdown("DRONE HAS LANDED")

  def land_callback(self,msg):
    self.landing = True

  def goal_callback(self, msg):
    if msg != None:
      self.isGoal = True

  def cmd_vel_callback(self,msg):
    if msg.linear.x == 0 and msg.linear.y == 0 and msg.linear.z == 0 and msg.angular.x == 0 and msg.angular.y == 0 and msg.angular.z == 0:
      self.twist_active = False
    else:
      self.twist_active = True

if __name__ == '__main__':
    TakeoffLand()