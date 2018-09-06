#!/usr/bin/python
import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Wrench, Twist
from sys import stdin

class Control():
	def __init__(self):
		self.rospy = rospy
		'''Parameters'''
		self.final_vel_topic = self.rospy.get_param("final_vel_topic","/turtle1/cmd_vel")
		self.force_topic = self.rospy.get_param("force_topic","/force")
		self.control_rate = self.rospy.get_param("control_rate",100)
		self.controller_params = {	"m": self.rospy.get_param("mass",8),
						"b_l": self.rospy.get_param("ldaming_ratio",10),
						"j": self.rospy.get_param("inertia",5),
						"b_a": self.rospy.get_param("adamping_ratio",20)
					}
		'''Control Params'''
		self.v_prima = self.v_anterior = self.v_actual = 0
		self.w_prima = self.w_anterior = self.w_actual = 0
		self.dt = self.last_time = 0
		self.limite_angular = 0.5
		self.limite_lineal = 0.5
		'''Subscribers'''
		self.sub_force = self.rospy.Subscriber(self.force_topic, Wrench, self.callback_force)
		'''Publisher'''
		self.pub_final_vel = self.rospy.Publisher(self.final_vel_topic, Twist, queue_size = 10)
		'''Node Configuration'''
		self.rospy.init_node('Control', anonymous = True)
		self.rate = self.rospy.Rate(self.control_rate)
		self.msg_force = Wrench()
		self.msg_vel = Twist()
		self.change = False
		self.main_control()

	def callback_force(self, data):
		#msg = self.vel_format(data)
		self.force = data.force.y
		self.torque = data.torque.z
		time = rospy.get_time()
		try:
			self.dt = time - self.last_time
			self.last_time = time
		except:
			self.dt = 0
			self.last_time = time
		print(self.dt)
		self.change = True
		return

	def get_response(self):
		'''Velocidad Lineal de Admitancia'''
		if self.force != 0:
			self.v_actual = (self.force - self.controller_params["m"]*self.v_prima)/self.controller_params["b_l"]
			self.v_prima = (self.v_actual - self.v_anterior)*self.dt
		else:
			self.v_actual = 0
		if self.v_actual > self.limite_lineal:
			self.v_actual = self.limite_lineal
		elif self.v_actual < -self.limite_lineal:
			self.v_actual = -self.limite_lineal
		self.v_anterior = self.v_actual
		'''Velocidad Angular de Admitancia'''
		if self.torque != 0:
			self.w_actual = (self.torque - self.controller_params["j"]*self.v_prima)/self.controller_params["b_a"]
			self.w_prima = (self.w_actual - self.w_anterior)*self.dt
		else:
			self.w_actual = 0
		if self.w_actual > self.limite_angular:
			self.w_actual = self.limite_angular
		elif self.w_actual < -self.limite_angular:
			self.w_actual = -self.limite_angular
		self.w_anterior = self.w_actual
		print('-'*50)
		print('Force', self.force, 'V lineal', self.v_actual)
		print('Torque', self.torque, 'V angular', self.w_actual)
		return

	def main_control(self):
		while not self.rospy.is_shutdown():
			if self.change:
				self.get_response()
				self.msg_vel.linear.x = self.v_actual
				self.msg_vel.angular.z = self.w_actual
				self.pub_final_vel.publish(self.msg_vel)
				self.change = False
			self.rate.sleep()

if __name__ == '__main__':
	try:
		sw = Control()
	except rospy.ROSInterruptException:
		pass
