#!/usr/bin/python
import rospy
import numpy as np
from std_msgs.msg import Bool, String, Float32
from geometry_msgs.msg import Wrench, Twist, Point
from sensor_msgs.msg import Joy
from std_srvs.srv import EmptyResponse, Empty
from rosfalcon.msg import falconSetPoint, falconPos, falconForces
from sys import stdin
from threading import Lock

class HapticTeleop():
	def __init__(self, name):
		self.name = name
		self.rospy = rospy
		self.rospy.init_node('HapticTeleop', anonymous = True)
		rospy.loginfo("Starting HapticTeleop Controller")
		self.initParameters()
		self.initSubscribers()
		self.initPublishers()
		self.initServiceClients()
		self.initVariables()
		self.mainControl()

	def initParameters(self):
		self.errorThetaTopic = self.rospy.get_param("~error_theta_topic", "/error_theta")
		self.setPointTopic = self.rospy.get_param("~set_point_topic", "/falcon/setpoint")
		self.setLEDTopic = self.rospy.get_param("~set_LED_topic", "/falcon/LED")
		self.falconPosTopic = self.rospy.get_param("~falcon_pos_topic", "/falcon/pos")
		self.falconFrcTopic = self.rospy.get_param("~falcon_force_topic", "/falcon/force")
		self.falconJoystickTopic = self.rospy.get_param("~falcon_joystick_topic", "/falcon/joystick")
		self.falconWrenchTopic = self.rospy.get_param("~falcon_wrench_topic", "/falcon_wrench")
		self.updateParamsService = self.name + self.rospy.get_param("~update_params_service","update_params")
		self.controlRate = self.rospy.get_param("~control_parameters/rate", 100)
		self.hapticMode = self.rospy.get_param("~control_parameters/mode", "LED") # LED - force
		self.controllerParams = {"k": self.rospy.get_param("~control_parameters/k",1),
								 "d": self.rospy.get_param("~control_parameters/d",0.1)}
		self.kVirtual = self.rospy.get_param("~k_virtual", 10)
		self.kSlope = self.rospy.get_param("~k_slope",100)
		self.param_lock = Lock()
		return

	def initSubscribers(self):
		self.subErrorTheta = self.rospy.Subscriber(self.errorThetaTopic, Float32, self.callbackErrorTheta)
		self.subfalconPos = self.rospy.Subscriber(self.falconPosTopic, falconPos, self.callbackFalconPos)
		self.subfalconJoy = self.rospy.Subscriber(self.falconJoystickTopic, Joy, self.callbackFalconJoystick)
		return

	def initPublishers(self):
		self.pubSetPoint = self.rospy.Publisher(self.setPointTopic, falconSetPoint, queue_size = 10)
		self.pubSetLED = self.rospy.Publisher(self.setLEDTopic, String, queue_size = 10)
		self.pubFalconForce = self.rospy.Publisher(self.falconFrcTopic, falconForces, queue_size = 10)
		self.pubFalconWrench = self.rospy.Publisher(self.falconWrenchTopic, Wrench, queue_size = 10)
		return

	def initServiceClients(self):
		self.rospy.Service(self.updateParamsService, Empty, self.callbackUpdateParams)
		return

	def initVariables(self):
		self.rate = self.rospy.Rate(self.controlRate)
		self.changeError = self.changePos = False
		self.xMax = 50
		self.angleMax = 120*np.pi/180
		self.xThreshold = 5
		self.posX = 0
		self.button = 0
		self.pressed = True
		return

	def callbackErrorTheta(self, msg):
		self.thetaError = msg.data
		if self.thetaError > self.angleMax:
			self.x = 50
		elif self.thetaError < -self.angleMax:
			self.x = -50
		else:
			self.x = self.xMax*self.thetaError/self.angleMax
		self.changeError = True
		return

	def callbackFalconPos(self, msg):
		self.posX = msg.X
		self.changePos = True
		return

	def callbackFalconJoystick(self, msg):
		self.button = msg.buttons[0]
		if self.button == 4 and not self.pressed:
			msgWrench = Wrench()
			msgWrench.torque.z = self.kVirtual*np.tanh(self.posX/self.kSlope)
			print(self.posX, msgWrench.torque.z)
			self.pubFalconWrench.publish(msgWrench)
			self.pressed = True
		else:
			if self.pressed:
				self.pressed = False
		return

	def callbackUpdateParams(self, req):
		with self.param_lock:
			self.initParameters()
			self.rospy.loginfo("Parameter update after request")
		return EmptyResponse()

	def makeMsgSetPoint(self):
		msg = falconSetPoint()
		msg.X = self.x
		self.pubSetPoint.publish(msg)
		return

	def makeMsgLED(self):
		self.msgLED = String()
		if self.posX >= self.xThreshold or self.posX <= -self.xThreshold:
			self.msgLED.data = "red"
		else:
			self.msgLED.data = "blue"
		self.pubSetLED.publish(self.msgLED)
		return

	def makeMsgForce(self):
		msg = falconForces()
		self.frc = np.sign(self.posX)*self.controllerParams["k"]*np.exp(np.power((-self.posX/self.controllerParams["d"]), 2))
		print(self.frc)
		msg.X = self.frc
		self.pubFalconForce.publish(msg)
		return

	def mainControl(self):
		rospy.loginfo("HapticTeleop Controller OK")
		while not self.rospy.is_shutdown():
			if self.changeError:
				self.makeMsgSetPoint()
				if self.changePos:
					if self.hapticMode == "LED":
						self.makeMsgLED()
					elif self.hapticMode == "force":
						self.makeMsgForce()
					else:
						self.rospy.loginfo("Invalid haptic mode")
			self.rate.sleep()

if __name__ == '__main__':
	try:
		sw = HapticTeleop('haptic_teleop')
	except rospy.ROSInterruptException:
		pass
