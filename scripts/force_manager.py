#!/usr/bin/python
import rospy
import numpy as np
import message_filters
from tf.transformations import quaternion_from_euler as qfe
from std_msgs.msg import Float32
from geometry_msgs.msg import WrenchStamped, Wrench

class ForceManager():
    def __init__(self):
        self.rospy = rospy
        '''Parameters'''
        self.frc_left_topic = self.rospy.get_param("~frc_left_topic","/frc_left")
        self.frc_right_topic = self.rospy.get_param("~frc_right_topic","/frc_right")
        self.error_theta_topic = self.rospy.get_param("~error_theta_topic","/error_theta")
        self.virtual_wrench_topic = self.rospy.get_param("~virtual_wrench_topic","/virtual_wrench")
        self.human_wrench_topic = self.rospy.get_param("~human_wrench_topic","/human_wrench")
        self.shared_wrench_topic = self.rospy.get_param("~shared_wrench_topic","/shared_wrench")
        self.d_sensors = self.rospy.get_param("~frc_sensors_distance", 0.8)
        self.k_virtual = self.rospy.get_param("~k_virtual_torque", 30)
        self.path_rate = self.rospy.get_param("~path_rate",100)
        '''Publisher'''
        self.pub_virtual_wrench = self.rospy.Publisher(self.virtual_wrench_topic, Wrench, queue_size = 10)
        self.pub_human_wrench = self.rospy.Publisher(self.human_wrench_topic, Wrench, queue_size = 10)
        self.pub_shared_wrench = self.rospy.Publisher(self.shared_wrench_topic, Wrench, queue_size = 10)
        '''Subscriber'''
        self.sub_frc_left = message_filters.Subscriber(self.frc_left_topic, WrenchStamped)
        self.sub_frc_right = message_filters.Subscriber(self.frc_right_topic, WrenchStamped)
        self.sub_error_theta = self.rospy.Subscriber(self.error_theta_topic, Float32, self.error_theta_callback)
        '''Synchronizer'''
        self.ts_frc = message_filters.TimeSynchronizer([self.sub_frc_left, self.sub_frc_right], 10)
        self.ts_frc.registerCallback(self.frc_callback)
        '''Node Configuration'''
        self.rospy.init_node('ForceManager', anonymous = True)
        self.rate = self.rospy.Rate(self.path_rate)
        self.msg_human_wrench = self.msg_virtual_wrench = self.msg_shared_wrench = Wrench()
        self.change1 = self.change2 = False
        self.main_path()

    def frc_callback(self, msg_left, msg_right):
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
        theta = msg_theta.data
        f1 = (1 - np.tanh(theta))*self.k_virtual
        f2 = (1 + np.tanh(theta))*self.k_virtual
        self.virtual_trq = (f2 - f1)*self.d_sensors/2
        self.msg_virtual_wrench.torque.z = self.virtual_trq
        self.pub_virtual_wrench.publish(self.msg_virtual_wrench)
        self.change2 = True
        return

    def make_shared_wrench(self):
        self.shared_trq = self.human_trq + self.virtual_trq
        self.msg_shared_wrench.torque.z = self.shared_trq
        self.pub_shared_wrench.publish(self.msg_shared_wrench)
        self.change1 = self.change2 = False
        return

    def main_path(self):
        rospy.loginfo("Force Manager OK")
        while not self.rospy.is_shutdown():
            if self.change1 and self.change2:
                self.make_shared_wrench()
            self.rate.sleep()

if __name__ == '__main__':
	try:
		print("Starting Force Manager")
		sw = ForceManager()
	except rospy.ROSInterruptException:
		pass
