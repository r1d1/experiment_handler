#!/usr/bin/python

'''
	NavExpManagerPy : python version of navigation experiment manager in order to go over catkin/rosbuild impatibilities ...

'''

import rospy
from std_msgs.msg import String, Bool, Float32
from BP_experiment.msg import StateReward, State, Actions
import roslib
roslib.load_manifest("lowlevel_actions") # In order to send goals, we need to use a package created by rosbuild and not catkin

class NavExpManager:
	def __init__(self):
		print "CTOR"
		self.statereward_sub = rospy.Subscriber("statereward", StateReward, self.statereward_cb)
		self.state_sub = rospy.Subscriber("statealone", State, self.state_cb)
		self.action_sub = rospy.Subscriber("actionToDo", Actions, self.action_cb)

		self.reward_pub = rospy.Publisher("rewardalone", Float32)
		self.learningMF_pub = rospy.Publisher("learningMF", Bool)
		self.learningMB_pub = rospy.Publisher("learningMB", Bool)
		self.learningMB2_pub = rospy.Publisher("learningMB2", Bool)

		self.robotState = State()
		self.robotState.stateID = "0"
		self.robotState.stateType = "Nav2"
		self.robotStateReward = StateReward()
		self.robotStateReward.stateID = self.robotState.stateID
		self.robotStateReward.stateType = self.robotState.stateType
		self.robotStateReward.reward = 0.0
		self.previousStateReward = self.robotStateReward
		
		self.robotStateChanged = False
		self.actionChanged = False

	def statereward_cb(self):
		self.previousStateReward = self.robotStateReward
		self.robotStateReward = msg
		self.robotStateChanged = True

	def state_cb(self):
		pass
	
	def action_cb(self):
		pass

if __name__ == "__main__":
	rospy.init_node("name")
	expHandler = NavExpManager()
	#expHandler.run()
	rospy.spin()
	print "Ending ..."
