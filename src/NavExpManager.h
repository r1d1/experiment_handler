#ifndef MODELFREE_H
#define MODELFREE_H

/* Class : NavExpManager
 *
 * @author : Erwan Renaudo
 * @created : 26/06/2015
 * @description : This class is a NavExpManager controller that associate states and actions with Qlearning rule.

 */

// General headers
#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <algorithm>
#include <fstream>
#include <getopt.h>

// ROS header
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Pose.h>

// Custom messages :
#include "habelar_msgs/StateReward.h"
#include "habelar_msgs/Actions.h"

#define NODECONTROLRATE 80

typedef std::pair<std::string, float> ActionReward;
typedef std::pair<std::string, std::string> StateAction;

class NavExpManager
{
	private :

		// Experience specific information :
		int durationOfExp; // how long (in iteration) will last the run ; 0 mean no ending
		std::string timeUnit; 

		int actioncount;
		long long int timecount;
		ros::Timer control_timer;
		ros::Time nodeStartTime;

		// Ros Node handler, subscribers and publishers :
		ros::NodeHandle nh_;

		ros::Subscriber state_sub;
		ros::Subscriber action_sub;
		
		// sending reward :
		ros::Publisher reward_pub;
		ros::Publisher learningMB_pub;
		ros::Publisher learningMB2_pub;
		ros::Publisher learningMF_pub;

		// Actions decided and executed :
		habelar_msgs::StateReward robotState;
		habelar_msgs::Actions actionDone;

		habelar_msgs::StateReward previousState;
		
		std_msgs::Float32 rewardToSend;
		//habelar_msgs::Reward rewardToSent;
		//------------------------------------------------
		bool robotStateChanged;
		bool actionChanged;
		bool doWeRun;

		//------------------------------------------------
		// State - Action - Reward Knowledge :
		//std::map<std::pair<std::string, std::string>, float> knownSAR;
		//std::map<std::string, ActionReward> knownSAR;
		std::map<StateAction, float> knownSAR;
		//------------------------------------------------
		// Reward obtained :
		float winReward;

		//------------------------------------------------
		// Files for logging
		std::ofstream stateevolution;
		std::ofstream reward_log;
		std::ofstream metainfo;
		//------------------------------------------------


	public :
		NavExpManager(ros::NodeHandle &nh, int exp, int duration, std::string timeunit, std::string fileToLoad); 
		~NavExpManager();

		bool run();

		void actionCallback(habelar_msgs::Actions msg);
		void stateCallback(habelar_msgs::StateReward msg);
		void positionCallback(geometry_msgs::Pose msg);

		void timerCallback(const ros::TimerEvent&);
};

#endif
