#!/usr/bin/env python

import rospy
from mavros_msgs.msg import PositionTarget, State
from geometry_msgs.msg import PoseStamped, Quaternion, Twist
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode, SetModeRequest
from std_msgs.msg import Empty
import sys, select, termios, tty

global uav

# get param from launch file and set default 
# value if variable is not set
flight_alt = rospy.get_param('~flight_alt', 0.75)
uav = rospy.get_param("/uav")

class Teleop():

	def __init__(self, altitude = flight_alt):

		rospy.init_node('quadsim_teleop') # creates the node

		self.state_sub = rospy.Subscriber("%s/mavros/state" % uav, State, self.state_cb)
		self.local_pose_pub = rospy.Publisher("%s/mavros/setpoint_position/local" % uav, PoseStamped, queue_size=10)

		self.landing = False

		# Clients
		self.arm_client = rospy.ServiceProxy("%s/mavros/cmd/arming" % uav, CommandBool)
		self.land_client = rospy.ServiceProxy("%s/mavros/cmd/land" % uav, CommandTOL)
		self.land_sub = rospy.Subscriber('/quadcopter_land', Empty, self.land_cb) # Listens for a command to land drone

		
		self.current_state = None
		self.des_z = altitude
		self.rate = rospy.Rate(20)
		self.arm()

		# subscribes to cmd_vel to receive velocity commands
		self.sub = rospy.Subscriber('/cmd_vel', Twist, self.twist_cb)

		# publishes velocity commands to hover and teleop
		self.pub = rospy.Publisher('%s/mavros/setpoint_raw/local' % uav, PositionTarget, queue_size=3)

		# define msg
		self.setvel_msg = PositionTarget()
		self.define_msg()
		
	def define_msg(self):
		self.setvel_msg.header.stamp = rospy.Time.now()
		self.setvel_msg.header.frame_id = "world"
		self.setvel_msg.coordinate_frame = 8 #"FRAME_BODY_NED"
		self.setvel_msg.type_mask = 64 | 128 | 256 | 1 | 2 | 1024 |512 
		# see http://docs.ros.org/jade/api/mavros_msgs/html/msg/PositionTarget.html
		self.setvel_msg.position.z = flight_alt
		self.setvel_msg.velocity.x = 0;
		self.setvel_msg.velocity.y = 0;
		self.setvel_msg.velocity.z = 0;
		self.setvel_msg.yaw_rate = 0
	
	def land_cb(self, msg):
		self.landing = True

	def state_cb(self, msg):
		self.current_state = msg 

	def twist_cb(self, msg):
		self.setvel_msg = PositionTarget()
		self.setvel_msg.header.stamp = rospy.Time.now()
		self.setvel_msg.header.frame_id = "world"
		self.setvel_msg.coordinate_frame = 8 #"FRAME_BODY_NED"
		self.setvel_msg.type_mask = 64 | 128 | 256 | 1 | 2 | 1024 |512 
		self.setvel_msg.velocity.x = msg.linear.x
		self.setvel_msg.velocity.y = msg.linear.y
		self.setvel_msg.velocity.z = msg.linear.z
		self.setvel_msg.position.z = flight_alt
		self.setvel_msg.yaw_rate = msg.angular.z

	def hover(self):
		self.setvel_msg = PositionTarget()
		self.setvel_msg.header.stamp = rospy.Time.now()
		self.setvel_msg.header.frame_id = "world"
		self.setvel_msg.coordinate_frame = 8 #"FRAME_BODY_NED"
		self.setvel_msg.type_mask = 64 | 128 | 256 | 1 | 2 | 1024 |512 
		self.setvel_msg.velocity.x = 0.
		self.setvel_msg.velocity.y = 0. 
		self.setvel_msg.velocity.z = 0.
		self.setvel_msg.position.z = flight_alt
		self.setvel_msg.yaw_rate = 0.

	def arm(self):
		# wait for connect
		while not rospy.is_shutdown() and self.current_state == None:
			rospy.loginfo("waiting for connection")
			self.rate.sleep()
		# must be streaming points before allowed to switch to offboard 
		pose = PoseStamped()
		pose.pose.position.x = 0
		pose.pose.position.y = 0
		pose.pose.position.z = self.des_z
		for i in range(100):
			self.local_pose_pub.publish(pose)
			self.rate.sleep()

		# change to offboard mode and arm
		last_request = rospy.get_time()
		# enable offboard mode 
		set_mode = rospy.ServiceProxy("%s/mavros/set_mode" % uav, SetMode)
		req = SetModeRequest()
		req.custom_mode = "OFFBOARD"
		while not rospy.is_shutdown() and (self.current_state.mode != req.custom_mode):
			self.local_pose_pub.publish(pose)
			if rospy.get_time() - last_request > 5.0: # check every 5 seconds
				try:
					set_mode.call(req)
				except rospy.ServiceException, e:
					print "Service did not process request: %s"%str(e)
				last_request = rospy.get_time()
			self.rate.sleep()
		rospy.loginfo("Switched to offboard mode")
		while not rospy.is_shutdown() and not self.current_state.armed:
			if not self.current_state.armed and rospy.get_time() - last_request > 5.0:
				if self.arm_client(True):
					rospy.loginfo("Vehicle armed")
				last_request = rospy.get_time()
			self.rate.sleep()

	def teleop(self):
		while not rospy.is_shutdown():
			self.pub.publish(self.setvel_msg)

			if self.landing:
				break

			self.rate.sleep()

		self.land()
		
	def land(self):
		while not self.land_client(0,0,0,0,0):
			rospy.loginfo("Vehicle landing")
			self.rate.sleep()

		rospy.loginfo("landed")
		rospy.signal_shutdown("Vehicle has landed")


if __name__=="__main__":
	tele = Teleop()
	tele.teleop()
	rospy.spin()