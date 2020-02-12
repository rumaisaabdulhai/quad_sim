#! /usr/bin/env python

'''
This python file is the code for allowing the drone to takeoff, hover, and land.

Author: Rumaisa Abdulhai
Date: February 2020

'''

#####################
# IMPORT STATEMENTS #
#####################
import rospy
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Empty
from move_base_msgs.msg import MoveBaseActionFeedback

#############
# CONSTANTS #
#############
DRONE_HEIGHT = 0.3

                                                            ############### BEG OF CLASS ###############
class TakeoffLand():

  ###############
  # CONSTRUCTOR #
  ###############
  def __init__(self):
    '''
    Initialization function for TakeoffLand Object.
    '''
    rospy.init_node('takeoff_land') # Creates the node

    # Instance Variables
    self.landing = False # If quadcopter in landing mode or not- initially set to false unless otherwise
    self.isGoal = False # If quadcopter has a current goal to reach - initially set to false unless otherwise
    self.twist_active = True # If quadcopter is currenty being given a command velocity - initially set to true unless otherwise
    self.twist_msg = Twist() # Twist message to publish for hovering

    # Subcribers
    self.land_sub = rospy.Subscriber('/move_base/feedback', MoveBaseActionFeedback, self.goal_cb) # Information about move_base goal
    self.cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_cb) # Allows access for current command velocities published
    self.takeoff_sub = rospy.Subscriber('/ground_truth_to_tf/pose', PoseStamped, self.takeoff_cb) # Information about pose of the drone
    self.land_sub = rospy.Subscriber('/quadcopter_land', Empty, self.land_cb) # Listens for a command to land drone

    # Publishers
    self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1) # For sending z command velocity for takeoff and hover
   
    self.rate = rospy.Rate(5) # rate at which to revisit callbacks
    rospy.spin() # keeps the node alive

  #############
  # CALLBACKS #
  #############

  def takeoff_cb(self,msg):
    '''
    This callback function takes in the current pose and other parameters to determine\n
    appropriate command velocities to send to the drone for taking off, hovering, or landing.

    Parameters:
    ----------
    self: The current object of type TakeoffLand.\n
    msg: The current pose of the drone in the global frame of type PoseStamped.
    '''

    z_pos = msg.pose.position.z # the current z position of the drone

    # If the drone is not landing or if the drone is not currently completing a 2D Nav goal
    if self.landing == False and self.isGoal == False:

      # If the current x and y velocities are 0
      if self.twist_msg.linear.x == 0 and self.twist_msg.linear.y == 0:
        # if the z_pos is between 1 and 1.3 meters, set a small z vel
        if z_pos > 1 and z_pos < 1.3:

          print ("                  SLIGHTLY LOW                  ")
          self.twist_msg.linear.z = 0.1

        # If the z_pos is between 1.3 and 1.5 meters, hover
        elif z_pos > 1.3 and z_pos < 1.5:
          print ("                    HOVERING                    ")
          self.twist_msg.linear.z = 0.0
        
        # If the z_pos is between 1.5 and 1.8 meters, set a small neg z vel
        elif z_pos > 1.5 and z_pos < 1.8:
          print ("                    SLIGHTLY HIGH                    ")
          self.twist_msg.linear.z = -0.1

        # If the z_pos is greater than 1.8 meters, set a med neg z vel
        elif z_pos > 1.8:
          print ("                    OVERSHOT                    ")
          self.twist_msg.linear.z = -0.3

        # If the drone is below 1 meter, takeoff with pos z vel
        else:
          print ("                    TAKING OFF                    ")
          self.twist_msg.linear.z = 0.3
      
        # Publish z velocity
        self.cmd_vel_pub.publish(self.twist_msg)

    # OR if the drone is completing a goal
    elif self.isGoal == True:

      # If a command velocity is not actually being set for the drone
      if self.twist_active == False:
        # The drone is actually not completing a goal
        self.isGoal = False

      # If a command velocity is being set for the drone
      else:
        print ("                    THERE IS A GOAL                    ")

    # OR if the drone is currently landing
    elif self.landing == True:

      global DRONE_HEIGHT
      # If the drone is above the ground
      if z_pos >= DRONE_HEIGHT:
        # Set a negative z velocity for the drone to come down and publish
        self.twist_msg.linear.z = -0.3
        self.cmd_vel_pub.publish(self.twist_msg)
        print ("                    LANDING                    ")

      # Once drone has reached ground, shut off takeoff_land node
      else:
        rospy.loginfo("System is shutting down. Stopping drone...")
        rospy.signal_shutdown("DRONE HAS LANDED")

  def land_cb(self,msg):
    '''
    This callback function listens for a message from the user if the drone\n
    must land and appropriately sets the instance variable landing to true.

    Parameters:
    ----------
    self: The current object of type TakeoffLand.\n
    msg: A blank message from the User that indicates the landing phase of the drone of type Empty.
    '''
    self.landing = True

  def goal_cb(self, msg):
    '''
    This callback function listens for a message from move_base/feedback to\n
    indicate whether the drone is currently working to complete a 2D navigation \n
    goal and appropriately sets the instance variable isGoal to true.

    Parameters:
    ----------
    self: The current object of type TakeoffLand.\n
    msg: A message that indicates the drone is still finishing a goal of type MoveBaseActionFeedback.
    '''
    self.isGoal = True

  def cmd_vel_cb(self,msg):
    '''
    This callback function listens to the current command velocities published\n
    and determines whether they are active or not based on the values in the\n
    command velocity message.

    Parameters:
    ----------
    self: The current object of type TakeoffLand.\n
    msg: The current command velocity being published of type Twist.
    '''
    # If all values in the cmd_vel message are 0, which means the drone is temporarily not moving
    if msg.linear.x == 0 and msg.linear.y == 0 and msg.linear.z == 0 and msg.angular.x == 0 and msg.angular.y == 0 and msg.angular.z == 0:
      # The command velocities are not active
      self.twist_active = False
    else:
      # The command velocities are active
      self.twist_active = True

                                                            ############### END OF CLASS ###############

if __name__ == '__main__':
    TakeoffLand() # Instantiates a TakeoffLand Object.