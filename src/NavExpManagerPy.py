#!/usr/bin/python

'''
	NavExpManagerPy : python version of navigation experiment manager in order to go over catkin/rosbuild impatibilities ...

'''

import rospy
from std_msgs.msg import String, Bool, Float32
from BP_experiment.msg import StateReward, State, Actions
import roslib
import sys
#print "Manifest not loaded, should be uncommented"
roslib.load_manifest("lowlevel_actions") # In order to send goals, we need to use a package created by rosbuild and not catkin
from optparse import OptionParser

class NavExpManager:
	def __init__(self, taskfile, rwdObj):
		print "CTOR"
		self.statereward_sub = rospy.Subscriber("statereward", StateReward, self.statereward_cb)
		self.state_sub = rospy.Subscriber("statealone", State, self.state_cb)
		self.action_sub = rospy.Subscriber("actionToDo", Actions, self.action_cb)

		self.reward_pub = rospy.Publisher("rewardalone", Float32)
		self.learningMF_pub = rospy.Publisher("learningMF", Bool)
		self.learningMB_pub = rospy.Publisher("learningMB", Bool)
		self.learningMB2_pub = rospy.Publisher("learningMB2", Bool)

		self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_cb) 
		
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

		self.rwdObj = rwdObj
		self.rwdAcc = 0.0

		self.timecount = 0
		self.actioncount = 0
		self.rewardcount = 0

		self.taskDescription= []
		self.taskfile = taskfile
		taskFile = open(self.taskfile, 'r+')
		text = taskFile.readlines()
		for i in range(len(text)):
			text[i] = text[i].strip('\n')
		taskFile.close()
		print len(text), "'",text,"'"
		if text[0] == "state":
			for line in text[1:]:
				elements = line.split()
				print elements[0], elements[1], elements[2]
				#self.taskDescription.append({"state":elements[0], "action":elements[1], "reward":elements[2]})
				# dict are shit in lists ...
				self.taskDescription.append(elements)
		elif text[0] == "target":
			for line in text[1:]:
				elements = line.split()
				print elements[0], elements[1], elements[2], elements[3]
				# See comment above
				self.taskDescription.append(elements)
				#self.taskDescription.append({"x":elements[0], "y":elements[1], "radius":elements[2],"reward":elements[3]})
		else:
			print "Task not understood !"
		#print self.taskDescription

	def statereward_cb(self, msg):
	#	self.previousStateReward = self.robotStateReward
		self.robotStateReward = msg
		self.robotStateChanged = True
		#print "SR Callback"
		#print "SR Callback", self.robotStateReward

	def state_cb(self, msg):
	#	print "S Callback"
		self.robotState = msg
#		self.stateChanged = True
	
	def action_cb(self, msg):
		self.actionDone = msg
		self.actionChanged = True
		#print "A Callback"
	
	def timer_cb(self, msg):
		self.timecount +i= 1
		# State machine :
		if expStatus == "monitor":
			if self.robotStateChanged and self.actionChanged:
				print "Was in", self.robotStateReward.stateID, "and did", self.actionDone.actionID
				td = zip(*self.taskDescription)
				#print "td 1: ", td[1]
				stateIndexes = [i for i, x in enumerate(td[0]) if x == self.robotStateReward.stateID]
				defstateIndexes = [i for i, x in enumerate(td[0]) if x == "all"]
				actionIndexes = [i for i, x in enumerate(td[1]) if (x == str(self.actionDone.actionID) or x == "all")]
				defactionIndexes = [i for i, x in enumerate(td[1]) if x == "all"]
				#print "cur, def:",stateIndexes, actionIndexes, defstateIndexes, defactionIndexes
				if not stateIndexes:
					state = defstateIndexes
					action = defactionIndexes 
				else:
					state = stateIndexes
					if not actionIndexes:
						action = defactionIndexes 
					else:
						action = actionIndexes
				lineToUse = list(set(state).intersection(action))
				#print "s,a,i:", state, action, lineToUse
				if not lineToUse:
					lineToUse[0] = 0
				rewardToSend=float(td[2][lineToUse[0]])
				self.rwdAcc += rewardToSend
				rwd = Float32()
				rwd.data = rewardToSend
				self.reward_pub.publish(rwd)
	
				print "reward:", rewardToSend
				
				self.monitorExp()
	
				self.robotStateChanged = False
				self.actionChanged = False

			if rewardToSend > 0.0:
				expStatus = "reset"
			#if self.rwdAcc >= self.rwdObj:
			#	expStatus = "reset"

		elif expStatus == "reset":
			if self.rwdAcc >= self.rwdObj:
				expStatus = "end"
			self.rwdAcc = 0.0
		elif expStatus == "end":
			# Exp finished, quit.
			


#	def resetExp(self):
		

if __name__ == "__main__":
	rospy.init_node("expManager")
	
	parser = OptionParser()
	parser.add_option("-f", "--file", action="store", type="string", dest="file")
	parser.add_option("-r", "--rwdObj", action="store", type="float", dest="rwdObj")
	(options, args) = parser.parse_args()
	print options, args, options.file, options.rwdObj
	
	expHandler = NavExpManager(options.file, options.rwdObj)
	rospy.spin()
	print "Ending ..."
