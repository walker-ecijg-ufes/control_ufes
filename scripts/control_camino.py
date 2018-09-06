#!/usr/bin/python
import rospy
import numpy as np
from nav_msgs.msg import Path
from sys import stdin

class PathControl():
	def __init__(self):
		self.rospy = rospy
		'''Parameters'''
		self.path_topic = self.rospy.get_param("~path_topic","/path")
        self.odom_topic = self.rospy.get_param("~odom_topic","/odom")
		self.route = self.rospy.get_param("~route","soft_turn_curve")
		self.path_rate = self.rospy.get_param("~path_rate",10)
		self.frame_id = self.rospy.get_param("~frame_id","odom")
		'''Publisher'''
		self.pub_path = self.rospy.Publisher(self.path_topic, Path, queue_size = 5)
		'''Node Configuration'''
		self.rospy.init_node('PathGeneration', anonymous = True)
		self.rate = self.rospy.Rate(self.path_rate)
		self.msg_path = Path()
		self.aux_header = Header()
		self.aux_pose_stamped = PoseStamped()
		self.seq = 0
		self.main_path()

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
		for i in range(self.xd.size):
			pose = Pose()
			pose.position.x = self.xd[i]
			pose.position.y = self.yd[i]
			pose.position.z = 0
			try:
				quat = qfe(0, 0, np.arctan2(self.yd[i+1]-self.yd[i],self.xd[i+1]-self.xd[i]))
			except:
				quat = qfe(0, 0, np.arctan2(self.yd[i]-self.yd[i-1],self.xd[i]-self.xd[i-1]))
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

	def main_path(self):
		exit = False
		while not self.rospy.is_shutdown() and not(exit):
			if self.route == 'straight':
				self.xd, self.yd = self.straight_line(0, 6, 0, 0.2)
				self.make_msg()
			elif self.route == 'soft_turn_curve':
				xd1, yd1 = self.straight_line(0, 2, 0, 0.2)
				xd2, yd2 = self.soft_turn_curve(2.2, 1, 0.2, 1)
				yd3, xd3 = self.straight_line(1, 5, xd2[-1], 0.2)
				self.xd = np.concatenate((xd1, xd2, xd3), axis=None)
				self.yd = np.concatenate((yd1, yd2, yd3), axis=None)
				self.make_msg()
			else:
				self.rospy.logwarn('Unknown route %s',  self.route)
			#exit = True
			self.rate.sleep()

if __name__ == '__main__':
	try:
		sw = PathGen()
	except rospy.ROSInterruptException:
		pass
