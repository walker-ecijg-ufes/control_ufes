#!/usr/bin/python
import rospy
import numpy as np
from tf.transformations import quaternion_from_euler as qfe
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from std_srvs.srv import EmptyResponse, Empty
from threading import Lock

class PathGen():
	def __init__(self, name):
		self.name = name
		self.rospy = rospy
		self.rospy.init_node('PathGeneration', anonymous = True)
		self.rospy.loginfo("[%s] Starting Path Generator", self.name)
		self.initParameters()
		self.initSubscribers()
		self.initPublishers()
		self.initServiceClients()
		self.initVariables()
		self.main_path()

	def initParameters(self):
		self.path_topic = self.rospy.get_param("~path_topic","/path")
		self.odom_topic = self.rospy.get_param("~odom_topic","/RosAria/pose")
		self.error_pos_topic = self.rospy.get_param("~error_pos_topic","/error_pos")
		self.theta_ref_topic = self.rospy.get_param("~theta_ref_topic","/theta_ref")
		self.route = self.rospy.get_param("~route","soft_turn_curve")
		self.updateParamsService = self.name + self.rospy.get_param("~update_params_service", "/update_parameters")
		self.path_rate = self.rospy.get_param("~path_rate", 10)
		self.frame_id = self.rospy.get_param("~frame_id","odom")
		self.param_lock = Lock()
		return

	def initSubscribers(self):
		self.sub_odom = self.rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback)
		return

	def initPublishers(self):
		self.pub_path = self.rospy.Publisher(self.path_topic, Path, queue_size = 5)
		self.pub_error_pos = self.rospy.Publisher(self.error_pos_topic, Point, queue_size = 5)
		self.pub_theta_ref = self.rospy.Publisher(self.theta_ref_topic, Float32, queue_size = 5)
		return

	def initServiceClients(self):
		self.rospy.Service(self.updateParamsService, Empty, self.callbackUpdateParams)
		return

	def initVariables(self):
		self.rate = self.rospy.Rate(self.path_rate)
		self.msg_path = Path()
		self.aux_header = Header()
		self.aux_pose_stamped = PoseStamped()
		self.point = Point()
		self.theta_ref = Float32()
		self.seq = self.x_bot = self.y_bot = 0
		self.change = False
		return

	def callbackUpdateParams(self, req):
		with self.param_lock:
			self.initParameters()
			self.rospy.loginfo("[%s] Parameter update after request", self.name)
		return EmptyResponse()

	def odom_callback(self, msg):
		self.x_bot = msg.pose.pose.position.x
		self.y_bot = msg.pose.pose.position.y
		self.change = True

	def straight_line(self, start_x, end_x, start_y, step):
		#step = 0.2
		xd = np.linspace(start_x, end_x, int((end_x-start_x)/step))
		yd = start_y*np.ones(xd.size)
		return xd, yd

	def soft_turn_curve(self, start_x, start_y, step, radius):
		#step = 0.15
		#radius = 1
		theta = np.linspace(3*np.pi/2, 2*np.pi, int(((2*np.pi)-(3*np.pi/2))/step))
		xd = start_x + radius*np.cos(theta)
		yd = start_y + radius*np.sin(theta)
		return xd, yd

	def make_msg(self):
		self.msg_path = Path()
		self.msg_path.header.seq = self.seq
		self.msg_path.header.stamp.secs = self.rospy.get_rostime().secs
		self.msg_path.header.stamp.nsecs = self.rospy.get_rostime().nsecs
		self.msg_path.header.frame_id = self.frame_id
		self.aux_header = self.msg_path.header
		#self.aux_pose_stamped.header = self.aux_header
		poses = []
		self.theta_path = []
		for i in range(self.xd.size):
			pose = Pose()
			pose.position.x = self.xd[i]
			pose.position.y = self.yd[i]
			pose.position.z = 0
			try:
				theta = np.arctan2(self.yd[i+1]-self.yd[i],self.xd[i+1]-self.xd[i])
			except:
				theta = np.arctan2(self.yd[i]-self.yd[i-1],self.xd[i]-self.xd[i-1])
			quat = qfe(0, 0, theta)
			self.theta_path.append(theta)
			pose.orientation.x = quat[0]
			pose.orientation.y = quat[1]
			pose.orientation.z = quat[2]
			pose.orientation.w = quat[3]
			self.aux_pose_stamped = PoseStamped()
			self.aux_pose_stamped.header = self.aux_header
			self.aux_pose_stamped.pose = pose
			poses.append(self.aux_pose_stamped)
		self.msg_path.poses = poses
		self.pub_path.publish(self.msg_path)
		self.seq += 1
		return

	def closest_point(self):
		self.point = Point()
		d = np.power((self.xd - self.x_bot),2) + np.power((self.yd - self.y_bot),2)
		loc = np.argmin(d)
		dmin = d[loc]
		xd_desired = self.xd[loc]
		yd_desired = self.yd[loc]
		if dmin < 0.04 and loc < self.xd.size:
			loc += 1
			xd_desired = self.xd[loc]
			yd_desired = self.yd[loc]
		if self.xd.size - loc <= 2:
			flag_end_of_path = 1
		else:
			flag_end_of_path = 0
		self.point.z = 0
		self.point.x = xd_desired - self.x_bot
		self.point.y = yd_desired - self.y_bot
		self.theta_ref.data = self.theta_path[loc]
		self.pub_error_pos.publish(self.point)
		self.pub_theta_ref.publish(self.theta_ref)

	def main_path(self):
		self.rospy.loginfo("[%s] PathGeneration OK", self.name)
		exit = False
		while not self.rospy.is_shutdown() and not(exit):
			if self.route == 'straight':
				self.xd, self.yd = self.straight_line(0, 6, 0, 0.2)
				flag_path = True
			elif self.route == 'soft_turn_curve':
				xd1, yd1 = self.straight_line(0, 2, 0, 0.2)
				xd2, yd2 = self.soft_turn_curve(2.2, 1, 0.2, 1)
				yd3, xd3 = self.straight_line(1, 5, xd2[-1], 0.2)
				self.xd = np.concatenate((xd1, xd2, xd3), axis=None)
				self.yd = np.concatenate((yd1, yd2, yd3), axis=None)
				flag_path = True
			else:
				self.rospy.logwarn('Unknown route %s',  self.route)
				flag_path = False
			if flag_path:
				self.make_msg()
				if self.change:
					self.closest_point()
					self.change = False
			#exit = True
			self.rate.sleep()

if __name__ == '__main__':
	try:
		sw = PathGen('path_gen')
	except rospy.ROSInterruptException:
		pass
