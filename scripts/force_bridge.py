#!/usr/bin/python
import rospy
from geometry_msgs.msg import Wrench
from geometry_msgs.msg import Twist
from sys import stdin

class Control():
	def __init__(self):
		self.rospy = rospy
		'''Parameters'''
		self.aux_vel_topic = self.rospy.get_param("aux_vel_topic","/aux_cmd_vel")
		self.force_topic = self.rospy.get_param("force_topic","/force")
		self.control_rate = self.rospy.get_param("switch_rate",100)
		'''Subscribers'''
		self.sub_vel = self.rospy.Subscriber(self.aux_vel_topic, Twist, self.callback_vel)
		'''Publisher'''
		self.pub_force = self.rospy.Publisher(self.force_topic, Wrench, queue_size = 10)
		'''Node Configuration'''
		self.rospy.init_node('Control', anonymous = True)
		self.rate = self.rospy.Rate(self.control_rate)
		self.msg_force = Wrench()
		self.msg_vel = Twist()
		self.change = False
		self.main_bridge()

	def callback_vel(self, data):
		#msg = self.vel_format(data)
		self.msg_force.force.x = 0
		self.msg_force.force.y = data.linear.x*4
		self.msg_force.force.z = 0
		self.msg_force.torque.x = 0
		self.msg_force.torque.y = 0
		self.msg_force.torque.z = data.angular.z*4
		self.change = True
		return

	def main_bridge(self):
		while not self.rospy.is_shutdown():
			if self.change:
				self.pub_force.publish(self.msg_force)
				self.change = False
			self.rate.sleep()

if __name__ == '__main__':
	try:
		sw = Control()
	except rospy.ROSInterruptException:
		pass
