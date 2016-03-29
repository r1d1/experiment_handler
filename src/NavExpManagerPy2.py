#!/usr/bin/python

'''
	NavExpManagerPy2 : python version of navigation experiment manager in order to go over catkin/rosbuild impatibilities ...

'''

import os
import rospy
import roslib
import sys
import math
import tf
import numpy as np
roslib.load_manifest("lowlevel_actions") # In order to send goals, we need to use a package created by rosbuild and not catkin # do we ?
from optparse import OptionParser

import actionlib
from std_msgs.msg import String, Bool, Float32
from BP_experiment.msg import StateReward, State, Actions, CommandSignal
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Point, PoseWithCovarianceStamped

class NavExpManager2:
	def __init__(self, taskfile, rwdObj, pose):
		print "CTOR"
		self.statereward_sub = rospy.Subscriber("statereward", StateReward, self.statereward_cb)
		self.state_sub = rospy.Subscriber("statealone", State, self.state_cb)
		self.action_sub = rospy.Subscriber("actionToDo", Actions, self.action_cb)
		self.actionFinished_sub = rospy.Subscriber("actionFinished", Bool, self.actionFinished_cb)
		self.pose_sub = rospy.Subscriber("robot_pose_ekf/odom_combined", PoseWithCovarianceStamped, self.pose_cb)

		self.reward_pub = rospy.Publisher("rewardalone", Float32)
		self.learningMF_pub = rospy.Publisher("learningMF", Bool)
		self.learningMB_pub = rospy.Publisher("learningMB", Bool)
		self.learningMB2_pub = rospy.Publisher("learningMB2", Bool)
		self.plde_pub = rospy.Publisher("/bp_experiment/commandChannel", CommandSignal)
		
		self.control_pub = rospy.Publisher("controlEnable", Bool)

		self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_cb) 
		# ----------------------------------
		self.goalTF = "/map"	
		self.goal = MoveBaseGoal()
		self.listener = tf.TransformListener()
		self.backToInit = True
		# ----------------------------------
		self.robotpose = [0.0, 0.0]

		self.robotState = State()
		self.robotState.stateID = "0"
		self.robotState.stateType = "Nav2"
		self.robotStateReward = StateReward()
		self.robotStateReward.stateID = self.robotState.stateID
		self.robotStateReward.stateType = self.robotState.stateType
		self.robotStateReward.reward = 0.0
		self.previousStateReward = self.robotStateReward
		self.robotStartStateRwd = self.robotStateReward

		self.robotStateChanged = False
		self.stateChanged = False
		self.actionChanged = False
		self.actionHasFinished = False
		#self.expStatus = "monitor"
		self.expState = 0
		self.expSubState = 0
		# ----------------------------------
		self.rwdObj = rwdObj
		self.rwdAcc = 0.0

		self.timecount = 0
		self.actioncount = 0
		self.rewardcount = 0

		self.taskType= ""
		self.taskDescription= []
		self.initialPoses= []
		self.initOrdPoses= []
		self.initRandPoses= []
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
		for line in poseTxt:
			self.initialPoses.append(line.split())
			for n in np.arange(5):
				self.initOrdPoses.append(self.initialPoses[-1])

		#print self.initOrdPoses
		while len(self.initOrdPoses)>0:
			self.initRandPoses.append(self.initOrdPoses.pop(np.random.randint(0,high=len(self.initOrdPoses))))
		print self.initRandPoses
		#print self.initOrdPoses
			
			
			
	 	# Process descriptions	
		self.taskType = taskTxt[0]
		if self.taskType == "state":
			for line in taskTxt[1:]:
				self.taskDescription.append(line.split())
		elif self.taskType == "goal":
			for line in taskTxt[1:]:
				self.taskDescription.append(line.split())
		elif self.taskType == "target":
			for line in taskTxt[1:]:
				self.taskDescription.append(line.split())
		else:
			print "Task not understood !"
		# ----------------------------------
		self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
		self.client.wait_for_server()
		# ----------------------------------
		# Start Exp :
		#print self.initialPoses
		print self.taskDescription
		print self.taskType
		# ----------------------------------

	def statereward_cb(self, msg):
		self.robotStateReward = msg
		self.robotStateChanged = True
		#print "SR Callback"
		print "SR Callback", self.robotStateReward.stateID, "(", self.robotStateReward.reward,")"

	def state_cb(self, msg):
		self.robotState = msg
		self.stateChanged = True
#		print "S Callback"
	
	def action_cb(self, msg):
		self.actionDone = msg
		self.actionChanged = True
		self.robotStartStateRwd = self.robotStateReward
#		print "A Callback", self.actionDone.actionID
	
	def actionFinished_cb(self, msg):
		self.actionHasFinished = msg.data
		print "AcF'd Callback", self.actionHasFinished
	
	def pose_cb(self, msg):
	#	print "pose,",msg
		try:
#			(position, quaternion) = self.listener.lookupTransform(self.goalTF, msg.header.frame_id, rospy.Time(0))
			(position, quaternion) = self.listener.lookupTransform(self.goalTF, "/base_link", rospy.Time(0))
		#	print position, quaternion
			self.robotpose[0] = position[0]
			self.robotpose[1] = position[1]
		except (tf.LookupException,tf.ConnectivityException,tf.ExtrapolationException) as e:
			rospy.loginfo("waiting for transform, %s",e)


	def timer_cb(self, msg):
		self.timecount += 1
		# State machine :
		# monitor, reset, end
		# monitor : experiment is running, we check the states and actions of the robot to provide reward
		# reset : goal has been reached, we stop learning and controller speed command and send a goal to drive the robot to an initial position. When at position, reactivate learning and control, send "action finished".
		# #end : enought reward accumulated, send shutdown signals to nodes, drive robot back to zero, emit sound ?
		#if self.expStatus == "monitor":
		if self.expState == 0:
			r = self.monitor()
			if r > 0.0:
				self.expState = 1 # reset
				print "Reset"
			elif self.rwdAcc >= self.rwdObj:
				# end
				self.expState = 2
			else:
				self.expState = 0
		elif self.expState == 1:
			self.reset()
		elif self.expState == 2:
			self.finish()
			# Exp finished, quit.
			
	def monitor(self):
		#if self.robotStateChanged and self.actionChanged:
		# When we got the action, we take the start state and compute reward
	#	print self.taskType
		rewardToSend = 0.0
		if self.taskType == "goal":
			if self.stateChanged:
				print "GG"
				rewardToSend = self.computeReward(self.robotState, None)
				self.rwdAcc += rewardToSend
				
				# publish reward message:
				rwd = Float32()
				rwd.data = rewardToSend
				self.reward_pub.publish(rwd)
		
				print "reward:", rewardToSend, ", total reward:", self.rwdAcc," (",100.0*self.rwdAcc/self.rwdObj,"%)"
		
				#self.robotStateChanged = False
				self.stateChanged = False
				self.actionChanged = False
				self.actionFinished = False
			#else:
			#	if self.actionHasFinished:
			#		self.actionHasFinished = False
		else :
			print "Error in task type !"
			exit()
		return rewardToSend

	def reset(self):
		#print "Reset"
		# reset exp: pause learning and control, send goal to base, wait until it's reached
		#print rospy.Time.now(), rospy.get_time()
		#if self.backToInit:
		if self.expSubState == 0:
			# sending planning and deciding inhibition
			planDecide = CommandSignal()
			planDecide.decide = False
			self.plde_pub.publish(planDecide)
			#keypressed = ""
			#while not (keypressed == " "):
			#	keypressed = raw_input()
					
			self.expSubState = 1

		elif self.expSubState == 1:
			print "Waiting for action to finish ...", self.actionHasFinished
		#	keypressed = ""
			#while not (keypressed == " "):
		#		keypressed = raw_input()
			if self.actionHasFinished:
				self.expSubState = 2

		elif self.expSubState == 2:	
			print "Action has finished:", self.actionHasFinished, " with reward, sending initial position."

			#randpose = self.initialPoses[np.random.randint(0, len(self.initialPoses))]
			randpose = self.initRandPoses.pop()
			# Compute angle:
			xr = self.robotpose[0]
			yr = self.robotpose[1]
			xg = float(randpose[0]) 
			yg = float(randpose[1])
			dx = xg - xr
			dy = yg - yr
			orientcos = dx / math.sqrt(dx * dx + dy * dy) if abs(dx+dy) > 0.0 else 1.0;
			orientsin = dy / math.sqrt(dx * dx + dy * dy) if abs(dx+dy) > 0.0 else 0.0;
			print "Rand pose:", randpose, "orient:", 180.0 * math.acos(orientcos) / 3.14159265359 , 180.0 * math.asin(orientsin) / 3.14159265359, 180.0 * math.atan2(dy, dx) / 3.14159265359 
			goalAngle = math.atan2(dy, dx)
			# Action lib goal
			print "AC goal"
			self.pauseSystem(True)

			self.goal = MoveBaseGoal()
			self.goal.target_pose.header.frame_id = self.goalTF
			self.goal.target_pose.header.stamp = rospy.Time.now()
			self.goal.target_pose.pose.position.x = float(randpose[0]) 
			self.goal.target_pose.pose.position.y = float(randpose[1]) 
			self.goal.target_pose.pose.orientation.x = 0.0 
			self.goal.target_pose.pose.orientation.y = 0.0
			self.goal.target_pose.pose.orientation.z = math.sin(goalAngle / 2.0)
			self.goal.target_pose.pose.orientation.w = math.cos(goalAngle / 2.0)
			
			print "eSS 2 "
		#	keypressed = ""
		#	while not (keypressed == " "):
		#		keypressed = raw_input()
	
			self.expSubState = 3

		elif self.expSubState == 3:	
	#		self.client.send_goal(self.goal)
			print "Waiting ..."
			#print self.client.get_goal_status_text(), self.client.get_state(), self.client.get_result()

			self.client.send_goal_and_wait(self.goal)
			print self.client.get_goal_status_text(), self.client.get_state(), self.client.get_result()
			if self.client.get_state():
				self.expSubState = 4

			#self.backToInit = False
			#self.actionHasFinished = False
				
		elif self.expSubState == 4:
			# Update expStatus:
			dx = self.goal.target_pose.pose.position.x - self.robotpose[0] 
			dy = self.goal.target_pose.pose.position.y - self.robotpose[1]
			dist = math.sqrt(dx*dx + dy*dy)
			print dist, self.client.get_goal_status_text(), self.client.get_state(), self.client.get_result(), actionlib.GoalStatus.ABORTED, actionlib.GoalStatus.SUCCEEDED
			goalStatus = self.client.get_state()
			if (goalStatus == actionlib.GoalStatus.SUCCEEDED):
				if self.rwdAcc < self.rwdObj:
					print "Back to monitoring ..."
					self.pauseSystem(False)
					self.expState = 0
				else:
					print "End of Experiment !"
					self.expState = 2
			elif (goalStatus == actionlib.GoalStatus.ABORTED):
				print "Something went wrong, drive the robot manually and then press Space!"
				keypressed = ""
				# wait for user's manual action:
				while not (keypressed == " "):
					keypressed = raw_input()
				print "Ok"
				self.pauseSystem(False)
				self.expState = 0
			else:
				print "Other status:", goalStatus

		#self.actionHasFinished = False
	

	def computeReward(self, st, act):
		#if msg.data:
		#	self.robotStartStateRwd = self.robotStateReward
		#print "Was in", self.robotStateReward.stateID, "and did", self.actionDone.actionID
		rwd = 0
		if self.taskType == "state":
			print "Was in", st.stateID, "and did", act.actionID
			td = zip(*self.taskDescription)
			# Check which state/action we did compared to task description:
			stateIndexes = [i for i, x in enumerate(td[0]) if x == st.stateID]
			defstateIndexes = [i for i, x in enumerate(td[0]) if x == "all"]
			actionIndexes = [i for i, x in enumerate(td[1]) if (x == str(act.actionID) or x == "all")]
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
			print "Task desc line:",lineToUse
			# If state and action are disjoint, use "all"' condition:
			if not lineToUse:
				lineToUse = [0]
				#lineToUse[0] = 0
			rwd=float(td[2][lineToUse[0]])
		elif self.taskType == "goal":
			print "I'm in ", st.stateID
			td = zip(*self.taskDescription)
			for i, x in enumerate(td[0]):
				stateIndexes = (x == st.stateID)
			if stateIndexes:
				rwd=float(self.taskDescription[0][1])
		return rwd 

	def pauseSystem(self, pausing):
		learningStatus = Bool()
		controlStatus = Bool()
		#if not pausing:
		#	self.expStatus = "monitor"
		#self.backToInit = not pausing

		learningStatus.data = not pausing
		self.learningMF_pub.publish(learningStatus)
		self.learningMB_pub.publish(learningStatus)
		self.learningMB2_pub.publish(learningStatus)
		controlStatus.data = not pausing
		self.control_pub.publish(controlStatus)
		self.askStopPlanning = not pausing
		planDecide = CommandSignal()
		planDecide.decide = not pausing
		self.plde_pub.publish(planDecide)

	def finish(self):
		print "Exp done"
		os.system("aplay /home/renaudo/Music/r2d2.wav")
		rospy.signal_shutdown("Exp ending !")

if __name__ == "__main__":
	rospy.init_node("expManager")
	
	parser = OptionParser()
	parser.add_option("-f", "--file", action="store", type="string", dest="file")
	parser.add_option("-p", "--poses", action="store", type="string", dest="poses")
	parser.add_option("-r", "--rwdObj", action="store", type="float", dest="rwdObj")
	(options, args) = parser.parse_args()
	print options, args, options.file, options.rwdObj
	
	expHandler = NavExpManager2(options.file, options.rwdObj, options.poses)
	rospy.spin()
	print "Ending ..."
