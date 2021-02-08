#!/usr/bin/env python

import rospy 
from geometry_msgs.msg import PoseStamped, Quaternion
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode, SetModeRequest
from mavros_msgs.msg import State
import time
from tf.transformations import quaternion_from_euler

flight_alt = 1.0 # (m)

class TakeOffLand():

	def __init__(self, altitude = flight_alt):

		rospy.init_node('takeoff_land') # creates the node

		# Subscribers
		self.state_sub = rospy.Subscriber("uav1/mavros/state", State, self.state_cb)

		# Publishers
		self.local_pose_pub = rospy.Publisher("uav1/mavros/setpoint_position/local", PoseStamped, queue_size=10)

		# Clients
		self.arm_client = rospy.ServiceProxy("uav1/mavros/cmd/arming", CommandBool)
		self.land_client = rospy.ServiceProxy("uav1/mavros/cmd/land", CommandTOL)
 
		self.current_state = None
		self.des_z = altitude
		self.rate = rospy.Rate(20)
		self.arm()

	def state_cb(self, msg):
		self.current_state = msg

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

		# enable offboard mode and arm
		last_request = rospy.get_time()
		set_mode = rospy.ServiceProxy("uav1/mavros/set_mode", SetMode)
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

	def take_off_hover(self):
		# define hover pose (set point)
		pose = PoseStamped()
		pose.header.stamp = rospy.get_rostime()
		pose.header.frame_id = 'mavsetp'
		pose.pose.position.x = 0
		pose.pose.position.y = 0
		pose.pose.position.z = self.des_z
		q = quaternion_from_euler(0, 0, 0)
		pose.pose.orientation = Quaternion(*q)
		rospy.loginfo("Vehicle taking off")
	
		# publish pose for however long we want to hover 
		while not rospy.is_shutdown():
			self.local_pose_pub.publish(pose)
			self.rate.sleep()
            
			rospy.loginfo("Vehicle hovering")

if __name__ == "__main__":
	takeoff_land = TakeOffLand()
	takeoff_land.take_off_hover()
	rospy.spin()