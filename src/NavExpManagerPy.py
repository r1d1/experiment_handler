#!/usr/bin/python

'''
	NavExpManagerPy : python version of navigation experiment manager in order to go over catkin/rosbuild impatibilities ...

'''

import rospy
import roslib
import sys
import numpy as np
roslib.load_manifest("lowlevel_actions") # In order to send goals, we need to use a package created by rosbuild and not catkin # do we ?
from optparse import OptionParser

import actionlib
from std_msgs.msg import String, Bool, Float32
from BP_experiment.msg import StateReward, State, Actions
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Point, PoseWithCovarianceStamped

class NavExpManager:
	def __init__(self, taskfile, rwdObj, pose):
		print "CTOR"
		self.statereward_sub = rospy.Subscriber("statereward", StateReward, self.statereward_cb)
		self.state_sub = rospy.Subscriber("statealone", State, self.state_cb)
		self.action_sub = rospy.Subscriber("actionToDo", Actions, self.action_cb)

		self.reward_pub = rospy.Publisher("rewardalone", Float32)
		self.learningMF_pub = rospy.Publisher("learningMF", Bool)
		self.learningMB_pub = rospy.Publisher("learningMB", Bool)
		self.learningMB2_pub = rospy.Publisher("learningMB2", Bool)
		
		self.control_pub = rospy.Publisher("controlEnable", Bool)

		self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_cb) 
		# ----------------------------------
		self.goalTF = "/map"	
		# ----------------------------------
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
		self.expStatus = "monitor"
		# ----------------------------------
		self.rwdObj = rwdObj
		self.rwdAcc = 0.0

		self.timecount = 0
		self.actioncount = 0
		self.rewardcount = 0

		self.taskDescription= []
		self.initialPoses= []
		# ----------------------------------
		self.taskfile = taskfile
		taskFile = open(self.taskfile, 'r+')
		taskTxt = taskFile.readlines()
		for i in range(len(taskTxt)):
			taskTxt[i] = taskTxt[i].strip('\n')
		taskFile.close()
		# ----------------------------------
		self.posefile = pose
		poseFile = open(self.posefile, 'r+')
		poseTxt = poseFile.readlines()
		for i in range(len(poseTxt)):
			poseTxt[i] = poseTxt[i].strip('\n')
		poseFile.close()
	 	# Process descriptions	
		if taskTxt[0] == "state":
			for line in taskTxt[1:]:
				self.taskDescription.append(line.split())

			for line in poseTxt:
				self.initialPoses.append(line.split())
	
		elif taskTxt[0] == "target":
			for line in taskTxt[1:]:
				self.taskDescription.append(line.split())
		else:
			print "Task not understood !"
		# ----------------------------------
		self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
		self.client.wait_for_server()
		# ----------------------------------
		# Start Exp :
		print self.initialPoses
		print self.taskDescription
		# ----------------------------------

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
		self.timecount += 1
		# State machine :
		# monitor, reset, end
		# monitor : experiment is running, we check the states and actions of the robot to provide reward
		# reset : goal has been reached, we stop learning and controller speed command and send a goal to drive the robot to an initial position. When at position, reactivate learning and control, send "action finished".
		# #end : enought reward accumulated, send shutdown signals to nodes, drive robot back to zero, emit sound ?
		if self.expStatus == "monitor":
			self.monitor()
		elif self.expStatus == "reset":
			self.reset()
		elif self.expStatus == "end":
			self.finish()
			# Exp finished, quit.
			
	def monitor(self):
		if self.robotStateChanged and self.actionChanged:
			print "Was in", self.robotStateReward.stateID, "and did", self.actionDone.actionID
			td = zip(*self.taskDescription)
			# Check which state/action we did compared to task description:
			stateIndexes = [i for i, x in enumerate(td[0]) if x == self.robotStateReward.stateID]
			defstateIndexes = [i for i, x in enumerate(td[0]) if x == "all"]
			actionIndexes = [i for i, x in enumerate(td[1]) if (x == str(self.actionDone.actionID) or x == "all")]
			defactionIndexes = [i for i, x in enumerate(td[1]) if x == "all"]
			
			if not stateIndexes:
				state = defstateIndexes
				action = defactionIndexes 
			else:
				state = stateIndexes
				if not actionIndexes:
					action = defactionIndexes 
				else:
					action = actionIndexes
			# Find the line to use to grab reward information:
			lineToUse = list(set(state).intersection(action))
			# If state and action are disjoint, use "all"' condition:
			if not lineToUse:
				lineToUse[0] = 0

			# Grab reward value:
			rewardToSend=float(td[2][lineToUse[0]])
			self.rwdAcc += rewardToSend

			# publish message: 
			rwd = Float32()
			rwd.data = rewardToSend
			self.reward_pub.publish(rwd)
	
			print "reward:", rewardToSend
	
			self.robotStateChanged = False
			self.actionChanged = False
			
			# Update expStatus:
			if rewardToSend > 0.0:
				self.expStatus = "reset"
			elif self.rwdAcc >= self.rwdObj:
				self.expStatus = "end"

	def reset(self):
		# reset exp: pause learning and control, send goal to base, wait until it's reached
		learningStatus = Bool()
		learningStatus.data = False
		self.learningMF_pub.publish(learningStatus)
		self.learningMB_pub.publish(learningStatus)
		self.learningMB2_pub.publish(learningStatus)
	
		controlStatus = Bool()
		controlStatus.data = False
		self.control_pub.publish(controlStatus)

		randpose = self.initialPoses[np.random.randint(0, len(self.initialPoses))]
		print "Rand pose:", randpose
		# Action lib goal
		print "AC goal"
		goal = MoveBaseGoal()
		goal.target_pose.header.frame_id = self.goalTF
		goal.target_pose.header.stamp = rospy.Time.now()
		goal.target_pose.pose.position.x = float(randpose[0]) 
		goal.target_pose.pose.position.y = float(randpose[0]) 
#		goal.target_pose.pose.position.z = target[2]
#		goal.target_pose.pose.orientation.x = target[3]
#		goal.target_pose.pose.orientation.y = target[4]
#		goal.target_pose.pose.orientation.z = target[5]
#		goal.target_pose.pose.orientation.w = target[6]
		#print goal
		self.client.send_goal(goal, feedback_cb=self.goalfeedback())
		print "Waiting ..."
		self.client.wait_for_result(rospy.Duration(10.0))
		# back to monitoring
		print "Monitoring"
		# Update expStatus:
		if self.rwdAcc >= self.rwdObj:
			self.expStatus = "end"
		else:
			self.expStatus = "monitor"
		
	def goalfeedback(self, fb):
		print fb

	def finish(self):
		print "Exp done"
		print('\a')
		rospy.signal_shutdown("Exp ending !")

if __name__ == "__main__":
	rospy.init_node("expManager")
	
	parser = OptionParser()
	parser.add_option("-f", "--file", action="store", type="string", dest="file")
	parser.add_option("-p", "--poses", action="store", type="string", dest="poses")
	parser.add_option("-r", "--rwdObj", action="store", type="float", dest="rwdObj")
	(options, args) = parser.parse_args()
	print options, args, options.file, options.rwdObj
	
	expHandler = NavExpManager(options.file, options.rwdObj, options.poses)
	rospy.spin()
	print "Ending ..."
