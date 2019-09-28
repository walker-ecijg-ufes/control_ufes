#!/usr/bin/python
import rospy
from geometry_msgs.msg import Twist

class SharedControl():
    '''This node will manage the incoming velocities
    from the control strategy developed by Wander et al
    and an admittance control strategy
    '''
    def __init__(self, name):
        self.name = name
        self.rospy = rospy
        self.rospy.init_node(self.name, anonymous = True)
        self.rospy.loginfo("[%s] Starting Shared Controller", self.name)
        self.initParameters()
        self.initSubscribers()
        self.initPublishers()
        self.initVariables()
        self.mainControl()

    def initParameters(self):
        self.finalVelTopic = self.rospy.get_param("~final_vel_topic", "/aux_cmd_vel")
        self.cognitiveVelTopic = self.rospy.get_param("~cognitive_vel_topic", "/cog_cmd_vel")
        self.physicalVelTopic = self.rospy.get_param("~physical_vel_topic", "/phy_cmd_vel")
        self.controlMode = self.rospy.get_param("~control_mode", 3) #1: only wander, #2: only admittance, #3: 50-50
        self.controlRate = self.rospy.get_param("~control_rate", 100)
        return

    def initSubscribers(self):
        if self.controlMode == 1:
            self.rospy.loginfo("[%s] Control mode '%s' is set", self.name, "Cognitive")
            self.subCmdVel =self.rospy.Subscriber(self.cognitiveVelTopic, Twist, self.callbackCmdVel)
        elif self.controlMode == 2:
            self.rospy.loginfo("[%s] Control mode '%s' is set", self.name, "Physical")
            self.subCmdVel =self.rospy.Subscriber(self.physicalVelTopic, Twist, self.callbackCmdVel)
        elif self.controlMode == 3:
            self.rospy.loginfo("[%s] Control mode '%s' is set", self.name, "Shared")
            self.subCmdVel1 =self.rospy.Subscriber(self.physicalVelTopic, Twist, self.callbackCmdVel1)
            self.subCmdVel2 =self.rospy.Subscriber(self.cognitiveVelTopic, Twist, self.callbackCmdVel2)
        else:
            self.rospy.logwarn("[%s] Invalid %s control mode. Using default '%s'", self.name, str(self.controlMode), "Shared")
            self.subCmdVel1 =self.rospy.Subscriber(self.physicalVelTopic, Twist, self.callbackCmdVel1)
            self.subCmdVel2 =self.rospy.Subscriber(self.cognitiveVelTopic, Twist, self.callbackCmdVel2)
            self.controlMode = 3
        return

    def initPublishers(self):
        self.pubFinalVel = self.rospy.Publisher(self.finalVelTopic, Twist, queue_size = 5)
        return

    def initVariables(self):
        self.msgVel = self.msgVel1 = self.msgVel2 = Twist()
        self.changeVel1 = self.changeVel2 = self.changeVel = False
        self.rate = self.rospy.Rate(self.controlRate)
        return

    def callbackCmdVel(self, msg):
        self.msgVel = msg
        self.changeVel = True
        return

    def callbackCmdVel1(self, msg):
        self.msgVel1 = msg
        self.changeVel1 = True
        return

    def callbackCmdVel2(self, msg):
        self.msgVel2 = msg
        self.changeVel2 = True
        return

    def makeVelMsg(self):
        self.msgVel = Twist()
        self.msgVel.linear.x = (self.msgVel1.linear.x + self.msgVel2.linear.x)/2
        self.msgVel.angular.z = (self.msgVel1.angular.z + self.msgVel2.angular.z)/2
        return

    def mainControl(self):
        self.rospy.loginfo("[%s] Shared Controller OK", self.name)
        while not self.rospy.is_shutdown():
            if self.controlMode == 1 or self.controlMode == 2:
                if self.changeVel:
                    self.pubFinalVel.publish(self.msgVel)
                    self.changeVel = False
            if self.controlMode == 3:
                if self.changeVel1 and self.changeVel2:
                    self.makeVelMsg()
                    self.pubFinalVel.publish(self.msgVel)
                    self.changeVel1 = self.changeVel2 = False

if __name__ == "__main__":
    try:
        control = SharedControl("SharedControl")
    except Exception as e:
        print("Something is wrong")
        print(e)
