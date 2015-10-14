/* File : NavExpManager.cpp

 * @author : Erwan Renaudo
 * @created : 27/06/2015 
 * @description : This node manages navigation experiment
 */

#include "NavExpManager.h"

NavExpManager::NavExpManager(ros::NodeHandle &nh, int experience, int duration=0, std::string timeunit="it", std::string loadFile="none")
{
	//------------------------------------------------------------------
	// ROS specific : topics, node handler
	nh_ = nh;
	// state and action topics
	state_sub = nh_.subscribe("state_publisher", 1, &NavExpManager::stateCallback, this);
	action_sub = nh_.subscribe("actionToDo", 1, &NavExpManager::actionCallback, this);
	//reward_pub = nh_.advertise<BP_experiment::Reward>(nodeDomain + "", 1);
	reward_pub = nh_.advertise<std_msgs::Float32>("reward_received", 1);
	// Control Learning :
	learningMF_pub = nh_.advertise<std_msgs::Bool>("bp_experiment/learningMF", 1);
	learningMB_pub = nh_.advertise<std_msgs::Bool>("bp_experiment/learningMB", 1);
	learningMB2_pub = nh_.advertise<std_msgs::Bool>("bp_experiment/learningMB2", 1);

	//------------------------------------------------------------------
	robotState.stateID = "0";
	robotState.stateType = "Nav2";
	robotState.reward = 0.0;
	robotStateChanged = false;
	previousState = robotState;

	// Reward to remember :
	winReward = 0.0;

	if( loadFile.compare("none") )
	{
		ROS_WARN("Loading task file : %s", loadFile.c_str());
		std::ifstream inFile;
		inFile.open(loadFile.c_str());
		// Processing task file :

		std::cout << "Processing input file " << std::endl;
		//std::string line;
		std::string state;
		std::string action;
		std::string taskDefType;
		float xpos, ypos, dist;
		float reward;
		inFile >> taskDefType;
		if( !taskDefType.compare("state") )
		{
			// Read the file
			while (inFile >> state >> action >> reward)
			{
				if( !state.compare("all") )
				{
					std::cout << "Default value for all states !" << std::endl;
				}
				if( !action.compare("all") ){ std::cout << "Default value for all actions !" << std::endl; }
				StateAction stac(state, action);
				knownSAR.insert(std::pair<StateAction, float>(stac, reward));
				std::cout << state << " - " << action << " - " << reward << std::endl << "--------------------" << std::endl;
			}
		}
		else
		{
			if( !taskDefType.compare("target") )
			{
				// Read the file
				while (inFile >> xpos >> ypos >> dist >> reward)
				{
//					StateAction stac(state, action);
					std::cout << xpos << ", " << ypos << " - " << dist << " - " << reward << std::endl << "--------------------" << std::endl;
				}
			}
			else
			{
				ROS_ERROR("File can not be understood, exiting ...");
				exit(EXIT_FAILURE);
			}
		}
		//std::cout << "... done" << std::endl;
		inFile.close();
	}

	//------------------------------------------------------------------

	durationOfExp = (duration > 0) ? duration : -1;
	control_timer = nh_.createTimer(ros::Duration(1.0 / NODECONTROLRATE), &NavExpManager::timerCallback, this);
	timeUnit = timeunit;
	actioncount = 0;
	timecount = 0;
	nodeStartTime = ros::Time::now();

	doWeRun = true;
}

NavExpManager::~NavExpManager(){}

//==========================================================================
// When we get the state and reward information, we update the internal State/Reward information :
void NavExpManager::stateCallback(const BP_experiment::StateReward msg)
{
	std::cout << "State Callback @ " << ros::Time::now() << " with msg @ " << msg << std::endl;
	robotState = msg;
	robotStateChanged = true;
}

//--------------------------------------------------------------------------

void NavExpManager::actionCallback(const BP_experiment::Actions msg)
{
	std::cout << "Action Callback" << std::endl;
	actionDone = msg;
	actionChanged = true;
	actioncount++;
}
//======================================================================

void NavExpManager::timerCallback(const ros::TimerEvent&)
{
	
	if(robotStateChanged && actionChanged)
	{
		//std::cout << "AC : " << actioncount << std::endl; 
		std::cout << "Was in : " << robotState.stateID << " and did " << actionDone.actionID << std::endl; 
		// Checking for the reward to send :
		std::ostringstream oss;
		oss << actionDone.actionID;
		std::string action = oss.str();

		//ActionReward currStac(robotState.stateID, action);

		StateAction currStac(robotState.stateID, action);
		std::map<StateAction, float>::iterator it = knownSAR.find(currStac);
		//std::map<std::string, ActionReward>::iterator it = knownSAR.find(robotState.stateID);
		//for(std::map<std::string, ActionReward>::iterator st=knownSAR.begin() ; st != knownSAR.end() ; st++){ std::cout << st->first << " - " << (st->second).first << " " << (st->second).second << std::endl;	}
		for(std::map<StateAction, float>::iterator st=knownSAR.begin() ; st != knownSAR.end() ; st++){ std::cout << (st->first).first << " - " << (st->first).second << " " << (st->second) << std::endl;	}

		if ( it != knownSAR.end() )
		{
			// If found, return the corresponding reward :
			rewardToSend.data = (it->second);
			//rewardToSend.data = (it->second).second;
			//std::cout << (it->second).first << " " << (it->second).second << std::endl;
			std::cout << (it->first).first << " " << (it->first).second << std::endl;

		}
		else
		{
			StateAction currStall(robotState.stateID, "all");
			std::map<StateAction, float>::iterator it_actionall = knownSAR.find(currStall);
			if ( it_actionall != knownSAR.end() )
			{
				// If we find a reward for a state and any action, return it :
				rewardToSend.data = (it_actionall->second);
			}
			else
			{
				// We don't know a specific reward, we send the all/all reward value :
				rewardToSend.data = knownSAR.at(StateAction("all", "all"));
			}
//			knownSAR.at("all")// If S-A not found, return the default reward :
		}
		std::cout << rewardToSend << std::endl;
		reward_pub.publish(rewardToSend);
		rewardToSend.data = 0.0;
		robotStateChanged=false;
		actionChanged=false;

	}
	timecount++;
	doWeRun = ( (!timeUnit.compare("dec")) ? (fabs(durationOfExp - actioncount)>0) : (fabs(durationOfExp - timecount)>0) ) ;
}

bool NavExpManager::run()
{
	while( nh_.ok() && doWeRun ){ ros::spinOnce(); }
	return true;
}

int main(int argc, char** argv)
{
	//init the ROS node
	ros::init(argc, argv, "navExpManager");
	ros::NodeHandle nh;

	int opt;
	// temp variables to store parameters, named according to options :
	int expe, dur;
	expe = dur = 0;
	std::string D="dec", load="none";
	int fixedSeed = -1;

	const struct option availableOptions[] = {
		{"help", no_argument, 0, 'h'},
		{"seed", optional_argument, NULL, 's'},
		{"load", optional_argument, NULL, 'L'},
		{"duration", optional_argument, 0, 'D'},
		{0,0,0,0}
	};

	while( (opt = getopt_long(argc, argv, "xdLbD:agh", availableOptions, NULL) ) != -1)
	{
		switch(opt)
		{
			case 'L' :
				// Set Seed for random values
				load = argv[optind];
				std::cout << " Load knowledge from : " << load << std::endl;
			break;
			case 's' :
				// Set Seed for random values
				fixedSeed = atoi(argv[optind]);
				std::cout << " Seed set : " << fixedSeed << std::endl;
			break;
			case 'x' : // set eXperience ID
				expe = atoi(argv[optind]);
				std::cout << " Experience : " << expe << std::endl;
			break;
			case 'd' : // set experience Duration
				dur = atoi(argv[optind]);
				std::cout << " Duration : " << dur << std::endl;
			break;
			case 'D' : // set experience Duration
				D = argv[optind];
				std::cout << " time unit : " << D << std::endl;
			break;
			case 'h' : // show Help option
				std::cout << " Help : " << std::endl
				<< "Usage : rosrun experimentHandler navExpManager <-h (help) | options>" << std::endl
				<< std::endl
				<< "-d \t\t : duration of a run (in iterations ; 0 means no duration limit)" << std::endl
				<< "-D \t\t : duration unit (\033[1mdec\033[21mision or \033[1mit\033[21meration)" << std::endl
				<< "-x \t\t : eXperience ID for logging files" << std::endl
				<< std::endl
				<< "-h \t\t : print this text and exit" << std::endl;
				exit(EXIT_SUCCESS);
			break;
			default :
				std::cout << "unknown option" << std::endl
				<< "This node expects some arguments, use " << std::endl
				<< "\t rosrun experimentHandler navExpManager -h" << std::endl
				<< "to get help." << std::endl;
				exit(EXIT_FAILURE);
			break;
		}
	}
	
	srand( ((fixedSeed == -1) ? time(NULL) : fixedSeed) );

	// Instantiate a NavExpManager Expert
	std::cout << "Starting Experiment \033[1m" << expe << "\033[21m for \033[1m" << dur << " " << D << "\033[21m using expfile \033[1m" << load << "\033[21m and seed \033[1m" << fixedSeed  << "\033[21m" << std::endl;
	NavExpManager manager(nh, expe, dur, D, load);
	// Start node
	manager.run();


	std::cout << "ending...." << std::endl;

	return EXIT_SUCCESS;
}
