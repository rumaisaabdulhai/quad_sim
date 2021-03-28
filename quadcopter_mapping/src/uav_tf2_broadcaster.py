#!/usr/bin/env python  
import rospy
 
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs.msg import PoseWithCovarianceStamped

global from_frame
global to_frame

uav = rospy.get_param('~uav')
from_frame = rospy.get_param('~from_frame')
to_frame = rospy.get_param('~to_frame')

class uavTFBroadcaster:

    def __init__(self, uav = uav):

         rospy.init_node('uav_tf2_broadcaster')
         self.pose_sub = rospy.Subscriber('%s/mavros/global_position/local' % uav, 
                                            PoseWithCovarianceStamped, self.transform_pose)


    def transform_pose(self, msg):

        # **Assuming /tf2 topic is being broadcasted
        tf_buffer = Buffer()
        listener = TransformListener(tf_buffer)

        pose_cov_stamped = PoseWithCovarianceStamped()
        pose_cov_stamped.pose = msg.pose
        pose_cov_stamped.header.frame_id = from_frame
        pose_cov_stamped.header.stamp = rospy.Time.now()

        try:
            # ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
            output_pose_stamped = tf_buffer.transform(pose_cov_stamped, to_frame, rospy.Duration(1))
            return output_pose_stamped.pose

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            raise

if __name__ == '__main__':
    uavTFBroadcaster()
    rospy.spin()