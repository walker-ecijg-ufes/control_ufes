#!/usr/bin/python
import rospy
import numpy as np
from tf.transformations import euler_from_quaternion as efq
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from std_msgs.msg import Float32
from geometry_msgs.msg import Point
from sys import stdin

class PathFollow():
	def __init__(self):
		self.rospy = rospy
		'''Parameters'''
		self.error_pos_topic = self.rospy.get_param("~error_pos_topic","/error_pos")
		self.error_theta_topic = self.rospy.get_param("~error_theta_topic","/error_theta")
		self.theta_ref_topic = self.rospy.get_param("~theta_ref_topic","/theta_ref")
		self.odom_topic = self.rospy.get_param("~odom_topic","/RosAria/pose")
		self.V_ref = self.rospy.get_param("~V_ref",0.5)
		self.ly = self.rospy.get_param("~Ly",3)
		self.lx = self.rospy.get_param("~Lx",3)
		self.kx = self.rospy.get_param("~Ky",0.7)
		self.ky = self.rospy.get_param("~Kx",0.7)
		self.theta_desired_topic = self.rospy.get_param("~theta_desired_topic","/theta_desired")
		self.path_rate = self.rospy.get_param("~path_rate",10)
		'''Publisher'''
		self.pub_theta_desired = self.rospy.Publisher(self.theta_desired_topic , Float32, queue_size = 5)
		self.pub_error_theta = self.rospy.Publisher(self.error_theta_topic , Float32, queue_size = 5)
		'''Subscriber'''
		self.sub_error_pos = self.rospy.Subscriber(self.error_pos_topic, Point, self.error_pos_callback)
		self.sub_theta_ref = self.rospy.Subscriber(self.theta_ref_topic, Float32, self.theta_ref_callback)
		self.sub_odom = self.rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback)
		'''Node Configuration'''
		self.rospy.init_node('PathFollowing', anonymous = True)
		self.rate = self.rospy.Rate(self.path_rate)
		self.error_pos = Point()
		self.error_theta = Float32()
		self.theta_ref = Float32()
		self.theta_desired = Float32()
		self.change1 = False
		self.change2 = False
		self.main_path()

	def odom_callback(self, msg):
		orientation = msg.pose.pose.orientation
		quat = [orientation.x, orientation.y, orientation.z, orientation.w]
		self.theta_bot = efq(quat)[2]
		self.change = True

	def error_pos_callback(self,msg):
		self.error_pos = msg
		self.change1 = True

	def theta_ref_callback(self,msg):
		self.theta_ref = msg.data
		self.change2 = True

	def control_path(self):
		self.msg_theta_desired = Float32()
		ytilde=self.error_pos.y
		xtilde=self.error_pos.x
		y_point = self.V_ref*np.sin(self.theta_ref)+ self.ly*np.tanh((self.ky/self.ly)*ytilde)
		x_point = self.V_ref*np.cos(self.theta_ref)+ self.lx*np.tanh((self.kx/self.lx)*xtilde)
		theta_desired = np.arctan2(y_point,x_point)
		self.theta_desired.data = theta_desired
		error_theta = self.theta_bot - theta_desired
		error_theta = np.mod((error_theta + np.pi), 2*np.pi) - np.pi
		self.error_theta.data = error_theta
		self.pub_theta_desired.publish(self.theta_desired)
		self.pub_error_theta.publish(self.error_theta)

	def main_path(self):
		rospy.loginfo("Path Follow OK")
		while not self.rospy.is_shutdown():
			if self.change1 and self.change2:
				self.control_path()
			self.rate.sleep()

if __name__ == '__main__':
	try:
		print("Starting Path Following")
		sw = PathFollow()
	except rospy.ROSInterruptException:
		pass
