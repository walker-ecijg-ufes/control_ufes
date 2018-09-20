#!/usr/bin/python
import rospy
import numpy as np
import time
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Wrench, Twist
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
		self.rospy.loginfo("Starting HapticTeleop Controller")
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
		self.hapticRate = self.rospy.get_param("~haptic_parameters/rate", 100)
		self.hapticMode = self.rospy.get_param("~haptic_parameters/mode", "free") # free - led - force
		self.hapticParams = {"frcGain": self.rospy.get_param("~haptic_parameters/frcGain",6),
							 "d": self.rospy.get_param("~haptic_parameters/d",0.3)}
		self.falconWrenchParams = {"kGain": self.rospy.get_param("~falcon_wrench/kGain", 10),	#Directly Proportional Gain
							  "kSlope": self.rospy.get_param("~falcon_wrench/kSlope",50)}	#Inversely Proportional Gain
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
		self.rate = self.rospy.Rate(self.hapticRate)
		self.changeError = self.changePos = self.changeJoy = False
		self.xMax = 50
		self.angleMax = 90*np.pi/180
		self.xThreshold = 10*np.pi/180
		self.posX = 0
		self.frc = 0
		self.button = 0
		self.invalidCount = 0
		self.exitFlag = False
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
		self.msgWrench = Wrench()
		self.button = msg.buttons[0]
		if self.button == 4:
			#Counter torque is achieved without minus sign
			if self.hapticMode == "free" or self.hapticMode == "force":
				self.msgWrench.torque.z = -1*self.falconWrenchParams["kGain"]*np.tanh(self.posX/self.falconWrenchParams["kSlope"])
			elif self.hapticMode == "LED":
				self.msgWrench.torque.z = 1*self.falconWrenchParams["kGain"]*np.tanh(self.posX/self.falconWrenchParams["kSlope"])
			else:
				self.msgWrench.torque.z = 0
			self.makeMsgForce()
		else:
			self.msgWrench.torque.z = 0
		self.changeJoy = True
		return

	def callbackUpdateParams(self, req):
		with self.param_lock:
			self.initParameters()
			self.checkMode()
			self.rospy.loginfo("Parameter update after request")
		return EmptyResponse()

	def makeMsgSetPoint(self):
		msg = falconSetPoint()
		msg.X = self.x
		self.pubSetPoint.publish(msg)
		return

	def makeMsgLED(self):
		msgLED = String()
		if self.hapticMode == "free":
			msgLED.data = "green"
		else:
			if self.thetaError >= self.xThreshold:
				#Error to the right
				msgLED.data = "red"
			elif self.thetaError <= -self.xThreshold:
				#Error to the left
				msgLED.data = "blue"
			else:
				#Error OK
				msgLED.data = "green" #blue
		self.pubSetLED.publish(msgLED)
		return

	def makeMsgWrench(self):
		self.pubFalconWrench.publish(self.msgWrench)
		return

	def makeMsgForce(self):
		msgForce = falconForces()
		if self.hapticMode == "free" or self.hapticMode == "LED":
			msgForce.X = 0
		elif self.hapticMode == "force":
			sign = np.sign(self.thetaError)
			msgForce.X = sign*(self.hapticParams["frcGain"]-self.hapticParams["frcGain"]*np.exp(-np.power((self.thetaError/self.hapticParams["d"]), 2)))
		else:
			msgForce.X = 0
		self.pubFalconForce.publish(msgForce)
		return

	def checkMode(self):
		modes = ["free", "LED", "force"]
		if self.hapticMode in modes:
			self.rospy.loginfo("Haptic mode %s is OK", self.hapticMode)
			self.invalidCount = 0
			self.exitFlag = False
			return True
		else:
			self.rospy.logwarn("Invalid '%s' Haptic mode - Retrying in 3 seconds", self.hapticMode)
			time.sleep(3)
			if self.invalidCount > 20:
				self.exitFlag = True
				self.rospy.logerr("Exiting HapticTeleop node due to invalid mode")
				return False
			else:
				self.invalidCount += 1
				return self.checkMode()
			return False

	def mainControl(self):
		rospy.loginfo("HapticTeleop OK")
		if self.checkMode():
			while not self.rospy.is_shutdown() and not self.exitFlag:
				if self.changeError:
					self.makeMsgSetPoint()
				if self.changePos and self.changeJoy:
					if self.hapticMode == "LED" or self.hapticMode == "free":
						self.makeMsgLED()
					self.makeMsgWrench()
					self.changeJoy = False
				self.rate.sleep()

if __name__ == '__main__':
	try:
		sw = HapticTeleop('haptic_teleop')
	except rospy.ROSInterruptException:
		pass
