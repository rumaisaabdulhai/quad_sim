#! /usr/bin/env python

'''
This python file is the code for recording the distance traveled and time taken
for the drone to navigate from a source point to a destination point.

Author: Rumaisa Abdulhai
Date: February 2020
'''

#####################
# IMPORT STATEMENTS #
#####################
import pandas as pd
import rospy
import math
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Empty
from actionlib_msgs.msg import GoalStatusArray

#############
# VARIABLES #
#############
GOAL_NUM = 1 # The goal number

def create_df():
    '''
    This method creates the Pandas dataframe.
    '''

    columns = [ 'Goal', 
                # 'Initial Position',
                'Final Position',
                'Elapsed Time',
                'Distance Traveled',
                'Average Speed' ]

    df = pd.DataFrame(columns= columns)
    return df

class Results():

    ###############
    # CONSTRUCTOR #
    ###############
    def __init__(self):
        '''
        Initialization function for Results object.
        '''
        rospy.init_node('quad_results') # Creates the node

        # Instance Variables
        self.isActive = False # If quadcopter has a current goal to reach - initially set to false unless otherwise
        self.isComplete = False

        self._df = create_df() # Pandas dataframe

        # POSITIONS
        # self.init_pos = None
        self.current_pos = None
        self.prev_pos = None

        # DISTANCE
        self.total_dist = 0

        # TIMES
        self.init_time = 0
        self.final_time = 0
        self.elapsed_time = 0

        self.avg_speed = 0

        # Subscribers
        self.pose_sub = rospy.Subscriber('/ground_truth_to_tf/pose', PoseStamped, self.pose_cb) # Information about pose of the drone
        self.status_sub = rospy.Subscriber('/move_base/status', GoalStatusArray, self.status_cb) # Status about the goal
        self.saver_sub = rospy.Subscriber('/results_saver', Empty, self.save_results_cb)

        self.rate = rospy.Rate(5) # rate at which to revisit callbacks
        rospy.spin() # keeps the node alive

    ####################
    # ACCESSOR METHODS #
    ####################
    def get_df(self):
        '''
        This method returns the Pandas dataframe.

        Parameters:
        ----------
        self: The current object of type Results.
        '''
        return self._df

    def get_delta_dist(self):

        # initial position x and y coordinates
        x_i = self.prev_pos.pose.position.x
        y_i = self.prev_pos.pose.position.y

        # final position x and y coordinates
        x_f = self.current_pos.pose.position.x
        y_f = self.current_pos.pose.position.y

        # Delta x and Delta y
        dx = x_f - x_i
        dy = y_f - y_i
        
        # Calculating euclidean distance
        dist = math.sqrt( math.pow(dx,2) + math.pow(dy,2) )
        return dist

    ###################
    # MUTATOR METHODS #
    ###################
    def reset_params(self):
        '''
        This method resets all the parameters to zero.

        Parameters:
        ----------
        self: The current object of type Results.
        '''

        self.isActive = False
        self.isComplete = False

        # self.init_pos = None
        self.current_pos = None
        self.prev_pos = None

        self.total_dist = 0

        self.init_time = 0
        self.final_time = 0
        self.elapsed_time = 0

        self.avg_speed = 0

        rospy.loginfo("Parameters have been reset.")

    def append_df(self):
        '''
        This method appends the information to the Pandas dataframe.

        Parameters:
        ----------
        self: The current object of type Results.
        '''
        global GOAL_NUM

        # x_i = round(self.init_pos.pose.position.x, 2)
        # y_i = round(self.init_pos.pose.position.y, 2)

        x_f = round(self.current_pos.pose.position.x, 2)
        y_f = round(self.current_pos.pose.position.y, 2)

        elapsed_time = round(self.elapsed_time)

        # init_pos = [x_i, y_i]
        final_pos = [x_f, y_f]

        total_dist = round(self.total_dist,2)
        avg_speed = round(self.avg_speed,3)
        
        self._df.at[ GOAL_NUM-1, 'Goal'] = GOAL_NUM
        # self._df.at[ GOAL_NUM-1, 'Initial Position'] = init_pos
        self._df.at[ GOAL_NUM-1, 'Final Position'] = final_pos
        self._df.at[ GOAL_NUM-1, 'Elapsed Time'] = elapsed_time
        self._df.at[ GOAL_NUM-1, 'Distance Traveled'] = total_dist
        self._df.at[ GOAL_NUM-1, 'Average Speed'] = avg_speed

        rospy.loginfo("Results have been appended to the dataframe.")

    def set_total_dist(self):
        dist = self.get_delta_dist()
        self.total_dist += dist
  
    def set_elapsed_time(self):
        '''
        Gets the elapsed time in seconds.

        Parameters:
        ----------
        self: The current object of type Results.
        '''
        self.elapsed_time = self.final_time - self.init_time

    def set_avg_speed(self):
        '''
        Sets the average speed in meters per second.

        Parameters:
        ----------
        self: The current object of type Results.
        '''
        self.avg_speed = self.total_dist / self.elapsed_time

    #############
    # CALLBACKS #
    #############
    def status_cb(self, msg):

        global GOAL_NUM

        if self.isComplete == False:

            if msg.status_list != []:

                self.init_time = msg.status_list[0].goal_id.stamp.secs

                if msg.status_list[0].status != 3:
                    self.isActive = True
                
                elif msg.status_list[0].status == 3:
                    self.isComplete = True
                    self.final_time = msg.header.stamp.secs

        else:

            self.set_elapsed_time()
            self.set_avg_speed()
            self.append_df()
            self.reset_params() # Resets all parameters
            GOAL_NUM+=1 # Ready to record the next goal

    def pose_cb(self, msg):

        if self.isActive == True:

            self.prev_pos = self.current_pos

            if self.prev_pos == None:
                self.prev_pos = msg
            
            self.current_pos = msg

            self.set_total_dist()

    def save_results_cb(self, msg):
        '''
        This method saves the Pandas DataFrame to a CSV File.

        Parameters:
        ----------
        self: The current object of type Results.\n
        msg: Indicator of saving results of type Empty.
        '''
        self.get_df().to_csv('~/catkin_ws/src/quadcopter_sim/quadcopter_results/src/Results.csv')
        rospy.loginfo("Results have been saved.")

if __name__ == '__main__':
  Results() # Instantiates a Results Object.