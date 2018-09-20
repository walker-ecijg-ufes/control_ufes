#!/usr/bin/python
import rospy
import numpy as np
import message_filters
from tf.transformations import quaternion_from_euler as qfe
from std_msgs.msg import Float32
from geometry_msgs.msg import WrenchStamped, Wrench
from std_srvs.srv import EmptyResponse, Empty
from threading import Lock

class ForceManager():
    def __init__(self, name):
        self.name = name
        self.rospy = rospy
        self.rospy.init_node('ForceManager', anonymous = True)
        self.rospy.loginfo("[%s] Starting ForceManager", self.name)
        self.initParameters()
        self.initSubscribers()
        self.initPublishers
        self.initServiceClients()
        self.initVariables()
        self.mainManager()

    def initParameters(self):
        self.frc_left_topic = self.rospy.get_param("~frc_left_topic","/frc_left")
        self.frc_right_topic = self.rospy.get_param("~frc_right_topic","/frc_right")
        self.error_theta_topic = self.rospy.get_param("~error_theta_topic","/error_theta")
        self.virtual_wrench_topic = self.rospy.get_param("~virtual_wrench_topic","/virtual_wrench")
        self.human_wrench_topic = self.rospy.get_param("~human_wrench_topic","/human_wrench")
        self.shared_wrench_topic = self.rospy.get_param("~shared_wrench_topic","/shared_wrench")
        self.falcon_wrench_topic = self.rospy.get_param("~falcon_wrench_topic","/falcon_wrench")
        self.d_sensors = self.rospy.get_param("~frc_sensors_distance", 0.8)
        self.k_virtual = self.rospy.get_param("~k_virtual_torque", 30)
        self.manager_rate = self.rospy.get_param("~manager_rate",100)
        self.updateParamsService = self.name + self.rospy.get_param("~update_params_service", "/update_parameters")
        self.param_lock = Lock()
        return

    def initSubscribers(self):
        self.sub_frc_left = message_filters.Subscriber(self.frc_left_topic, WrenchStamped)
        self.sub_frc_right = message_filters.Subscriber(self.frc_right_topic, WrenchStamped)
        self.sub_error_theta = self.rospy.Subscriber(self.error_theta_topic, Float32, self.error_theta_callback)
        self.sub_falcon_wrench = self.rospy.Subscriber(self.falcon_wrench_topic, Wrench, self.falcon_wrench_callback)
        return

    def initPublishers(self):
        self.pub_virtual_wrench = self.rospy.Publisher(self.virtual_wrench_topic, Wrench, queue_size = 10)
        self.pub_human_wrench = self.rospy.Publisher(self.human_wrench_topic, Wrench, queue_size = 10)
        self.pub_shared_wrench = self.rospy.Publisher(self.shared_wrench_topic, Wrench, queue_size = 10)
        self.ts_frc = message_filters.TimeSynchronizer([self.sub_frc_left, self.sub_frc_right], 10)
        self.ts_frc.registerCallback(self.frc_callback)
        return

    def initServiceClients(self):
		self.rospy.Service(self.updateParamsService, Empty, self.callbackUpdateParams)
		return

    def initVariables(self):
        self.rate = self.rospy.Rate(self.manager_rate)
        self.virtual_trq = 0
        self.change1 = self.change2 = self.change3 = False
        return

    def callbackUpdateParams(self, req):
		with self.param_lock:
			self.initParameters()
			self.rospy.loginfo("[%s] Parameter update after request", self.name)
		return EmptyResponse()

    def frc_callback(self, msg_left, msg_right):
        self.msg_human_wrench = Wrench()
        frc_left = msg_left.wrench.force.y
        frc_right = msg_right.wrench.force.y
        self.human_frc = (frc_left + frc_right)/2
        self.human_trq = ((frc_right - frc_left)/2)*self.d_sensors
        self.msg_human_wrench.force.y = self.human_frc
        self.msg_human_wrench.torque.z = self.human_trq
        self.pub_human_wrench.publish(self.msg_human_wrench)
        self.change1 = True
        return

    def error_theta_callback(self, msg_theta):
        self.msg_virtual_wrench = Wrench()
        theta = msg_theta.data
        f1 = (1 + np.tanh(theta))*self.k_virtual
        f2 = (1 - np.tanh(theta))*self.k_virtual
        self.virtual_trq = (f2 - f1)*self.d_sensors/2
        self.msg_virtual_wrench.torque.z = self.virtual_trq
        self.pub_virtual_wrench.publish(self.msg_virtual_wrench)
        self.change2 = True
        return

    def falcon_wrench_callback(self, msg_falcon):
        self.falcon_trq = msg_falcon.torque.z
        self.change3 = True
        return

    def make_shared_wrench(self):
        self.msg_shared_wrench = Wrench()
        self.shared_trq = self.human_trq + self.virtual_trq + self.falcon_trq
        self.msg_shared_wrench.force.y = self.human_frc
        self.msg_shared_wrench.torque.z = self.shared_trq
        self.pub_shared_wrench.publish(self.msg_shared_wrench)
        self.change1 = self.change2 = self.change3 = False
        return

    def mainManager(self):
        self.rospy.loginfo("[%s] Force Manager OK", self.name)
        while not self.rospy.is_shutdown():
            if self.change1 and self.change2 and self.change3:
                self.make_shared_wrench()
            self.rate.sleep()

if __name__ == '__main__':
	try:
		sw = ForceManager('force_manager')
	except rospy.ROSInterruptException:
		pass
