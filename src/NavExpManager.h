#ifndef MODELFREE_H
#define MODELFREE_H

/* Class : NavExpManager
 *
 * @author : Erwan Renaudo
 * @created : 05/01/2014
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
#include "BP_experiment/StateReward.h"
#include "BP_experiment/Actions.h"

#define NODECONTROLRATE 80

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

		// Actions decided and executed :
		BP_experiment::StateReward robotState;
		BP_experiment::Actions decision;

		BP_experiment::StateReward previousState;
		//------------------------------------------------
		bool stateChanged;
		bool doWeRun;

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

		void actionCallback(BP_experiment::Actions msg);
		void stateCallback(BP_experiment::StateReward msg);
		void positionCallback(geometry_msgs::Pose msg);

		void timerCallback(const ros::TimerEvent&);
};

#endif
