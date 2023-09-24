#!/usr/bin/env python3

import math
import rospy
import tf
import pymap3d

from geometry_msgs.msg import Pose, Quaternion
from gazebo_msgs.msg import ModelStates
from geographic_msgs.msg import GeoPoint
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry

### main #####################################################################

def main():
	mavrosSpoofer().loop()

class mavrosSpoofer:
	def __init__(self):

		rospy.init_node("mavrosSpoofer")

		### local variables ##################################################

		self.local_position_origin = None
		self.global_origin = None

		### connect to ROS ###################################################

		sim_local_poses_topic   = rospy.get_param("~sim_local_poses_topic")
		local_position_topic    = rospy.get_param("~local_position_topic")
		global_position_topic   = rospy.get_param("~global_position_topic")
		global_origin_topic     = rospy.get_param("~global_origin_topic")
		
		default_lat	= rospy.get_param("~default_lat")
		default_lon = rospy.get_param("~default_lon")

		self.global_origin	 = GeoPoint(
			latitude=default_lat,
			longitude=default_lon,
			altitude=0,
		)

		self.sim_robot_name     = rospy.get_param("~sim_robot_name")


		self.local_pose_pub     = rospy.Publisher(local_position_topic, Pose, queue_size=1)
		self.global_pose_pub    = rospy.Publisher(global_position_topic, GeoPoint, queue_size=1)
		self.global_origin_pub  = rospy.Publisher(global_origin_topic, GeoPoint, queue_size=1)

		self.sim_local_pose_sub     = rospy.Subscriber('/' + self.sim_robot_name + sim_local_poses_topic, Odometry, self.sim_local_callback)

		### end init #########################################################

	### callbacks ############################################################

	def sim_local_callback(self, odom: Odometry):
		raw_local_position  = odom.pose.pose.position
		if not self.local_position_origin:
			self.local_position_origin = raw_local_position

		corrected_pose = Pose()
		corrected_pose.orientation  = odom.pose.pose.orientation
		corrected_pose.position.x   = raw_local_position.x - self.local_position_origin.x
		corrected_pose.position.y   = raw_local_position.y - self.local_position_origin.y
		corrected_pose.position.z   = raw_local_position.z - self.local_position_origin.z

		m_x 	= corrected_pose.position.x
		m_y 	= corrected_pose.position.y
		m_z 	= corrected_pose.position.z
		lat0	= self.global_origin.latitude
		lon0	= self.global_origin.longitude
		h0		= self.global_origin.altitude

		global_pose = pymap3d.enu2geodetic(m_x, m_y, m_z, lat0, lon0, h0, ell=None, deg=True)

		formatted_global_pose	 = GeoPoint(
			latitude=global_pose[0],
			longitude=global_pose[1],
			altitude=global_pose[2],
		)

		self.local_pose_pub.publish(corrected_pose)
		self.global_pose_pub.publish(formatted_global_pose)
		self.global_origin_pub.publish(self.global_origin)

	# def sim_global_pose_callback(self, gps_nsf: NavSatFix):
	# 	gps_gp = GeoPoint(
	# 		latitude=gps_nsf.latitude,
	# 		longitude=gps_nsf.longitude,
	# 		altitude=gps_nsf.altitude,
	# 	)
	# 	if not self.global_origin:
	# 		self.global_origin = gps_gp
	# 	self.global_pose_pub.publish(gps_gp)
	# 	self.global_origin_pub.publish(self.global_origin)	

	def loop(self):
		rospy.spin()

if __name__ == "__main__":
	main()
