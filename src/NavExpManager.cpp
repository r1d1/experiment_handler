/* File : NavExpManager.cpp

 * @author : Erwan Renaudo
 * @created : 27/06/2015 
 * @description : This node learns the regularities of robot actions in the block push experiment.
 */

#include "NavExpManager.h"

NavExpManager::NavExpManager(ros::NodeHandle &nh, int experience, int duration=0, std::string timeunit="it", std::string loadFile="none")
{
	//------------------------------------------------------------------
	// ROS specific : topics, node handler
	nh_ = nh;
	// state and action topics
	state_sub = nh_.subscribe("statereward", 1, &NavExpManager::stateCallback, this);
	action_sub = nh_.subscribe("habitualAction", 1, &NavExpManager::actionCallback, this);
	//reward_pub = nh_.advertise<BP_experiment::Reward>(nodeDomain + "", 1);
	reward_pub = nh_.advertise<std_msgs::Float32>("reward_received", 1);

	//------------------------------------------------------------------
	robotState.stateID = "0";
	robotState.stateType = "";
	robotState.contact = 0;
	robotState.view = 0;
	robotState.reward = 0.0;
	stateChanged = false;
	previousState = robotState;

	// Reward to remember :
	winReward = 0.0;

	if( loadFile.compare("none") )
	{
		ROS_WARN("Ensure you provided state and action space size consistent with the log file you want to load ");
		std::ifstream inFile;
		inFile.open(loadFile.c_str());
//		inFile >> defaultWeights[i][j];
		inFile.close();
	}

	// Initializing to 1st action may be a small bias :
	//actionDone.source = "MF";
	//actionDone.actionID = 0;

	//------------------------------------------------------------------

	durationOfExp = (duration > 0) ? duration : -1;
	control_timer = nh_.createTimer(ros::Duration(1.0 / NODECONTROLRATE), &NavExpManager::timerCallback, this);
	timeUnit = timeunit;
	actioncount = 0;
	timecount = 0;
	nodeStartTime = ros::Time::now();

	//------------------------------------------------------------------
	// Logs :
	//------------------------------------------------------------------
	// Version conversion in string
	/*std::ostringstream oss;
	oss << VERSION;
	std::string version = oss.str();

	// experience ID conversion in string
	oss.str(std::string());
	oss << experience;
	std::string exp_id = oss.str();

	// Path for logging
	std::string path = std::string("logs/modelfree/");
	std::string fileprefix = (path + "v" + version + "_exp" + exp_id + "_MF_");
	*/
	//------------------------------------------------------------------
	// File opening
	/*actionactivity_log.open(( fileprefix + std::string("actionactivity_log.dat")).c_str(), std::ios::out|std::ios::trunc);
	actionprob_log.open(( fileprefix + std::string("actionprob_log.dat")).c_str(), std::ios::out|std::ios::trunc);

	for(int act = 0 ; act < outputSize ; act++)
	{
		oss.str(std::string());
		oss << act;
		Weights_log[act].open( ( fileprefix + std::string("weights_act") + oss.str() + std::string("_log.dat") ).c_str(), std::ios::out|std::ios::trunc);
	// ---------------------------------------------------------------------------------------------------------------------------

	biasWeights_log.open(( fileprefix + std::string("biasWeights_log.dat")).c_str(), std::ios::out|std::ios::trunc);

	stateevolution.open(( fileprefix + std::string("stateevolution_log.dat")).c_str(), std::ios::out|std::ios::trunc);
	reward_log.open(( fileprefix + std::string("reward_log.dat")).c_str(), std::ios::out|std::ios::trunc);

	deltaWeights_log.open(( fileprefix + std::string("deltaWeights_log.dat")).c_str(), std::ios::out|std::ios::trunc);

	metainfo.open(( fileprefix + std::string("metainfo_log.dat")).c_str(), std::ios::out|std::ios::trunc);

	if(!deltaWeights_log | !biasWeights_log | !metainfo |
	!stateevolution | !reward_log | !actionactivity_log | !actionprob_log)  // si l'ouverture a plante
	{
		ROS_ERROR("log file not opening, abort ...");
		exit(EXIT_FAILURE);
   	}

	metainfo << "VERSION " << VERSION << std::endl
		<< "TEMPERATURE " << softmax_temperature << std::endl
		<< "TIMEWINDOW " << TIMEWINDOW << std::endl
		<< "ACTIONSTATESIZE " << ACTIONSTATESIZE << std::endl
		<< "OUTPUTSIZE " << outputSize << std::endl
		<< "GAMMA " << discount_rate << std::endl
		<< "ALPHA " << learning_rate << std::endl
		<< "DNTH_REWARD " << DNTH_REWARD << std::endl
		<< "CAM_REWARD " << CAM_REWARD << std::endl
		<< "ARM_REWARD " << ARM_REWARD << std::endl
		<< "SUCCESS_RWD " << SUCCESS_RWD << std::endl
		<< "BELT_LENGTH " << BELT_LENGTH << std::endl
		<< "NODECONTROLRATE " << NODECONTROLRATE << std::endl
		<< "BELT_SPEED " << BELT_SPEED << std::endl
		<< "PARAM_BLOCK_SPACE " << PARAM_BLOCK_SPACE << std::endl
		<< "VARIANCE_SPACE " << VARIANCE_SPACE << std::endl;

		std::cout << "MF CTOR done. Ready." <<std::endl;
		doWeRun = true;
		// We sleep to give time to subscription to happen :
		sleep(1);
		std::cout << "connected to : " << state_sub.getNumPublishers()<< std::endl;
		// If no sub, we wait :
		while(!state_sub.getNumPublishers()){ sleep(1); }
		// Sending first action to start getting feedback :
		actions_pub.publish(actionDone);
	*/
}

NavExpManager::~NavExpManager()
{
	/*metainfo << "DURATION " << actioncount << std::endl
		 << "DURATION_REQ " << durationOfExp << std::endl;
	//------------------------------------------------------------------
	// Closing log file :

	actionactivity_log.close();
	actionprob_log.close();

	for(int act = 0 ; act < outputSize ; act++){ Weights_log[act].close(); }

	biasWeights_log.close();

	stateevolution.close();
	reward_log.close();

	deltaWeights_log.close();

	metainfo.close();*/
	//------------------------------------------------------------------
}

//==========================================================================

// When we get the state and reward information, we update the internal State/Reward information :
void NavExpManager::stateCallback(const BP_experiment::StateReward msg)
{
	// New state management (to be used) :
	robotState = msg;
	stateChanged = true;
	//reward_log << timecount << " " << actioncount << " " << winReward << " " << ros::Time::now() - nodeStartTime << std::endl;
}

void NavExpManager::actionCallback(const BP_experiment::Actions msg){}
//======================================================================

void NavExpManager::timerCallback(const ros::TimerEvent&)
{
	if( stateChanged )
	{
		std::cout << "AC : " << actioncount << std::endl; 
		reward_pub.publish(decision);
		stateChanged=false;

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
	int x, d, c;
	x = d = 0;
	float a, g, t, s;
	a = 0.1,  g = 0.9, t = 0.5;
	s = 0.1;
	std::string D, L="none";

	int fixedSeed = -1;
	int insize=1, outsize=1;

	const struct option availableOptions[] = {
		{"help", no_argument, 0, 'h'},
		{"seed", optional_argument, NULL, 'b'},
		{"load", optional_argument, NULL, 'L'},
		{"statesize", optional_argument, NULL, 'S'},
		{"actionnumber", optional_argument, NULL, 'A'},
		{"duration", optional_argument, 0, 'D'},
		{0,0,0,0}
	};

	while( (opt = getopt_long(argc, argv, "xdLtbD:agh", availableOptions, NULL) ) != -1)
	{
		switch(opt)
		{
			case 'S' :
				// Set input state size
				insize = atoi(argv[optind]);
				std::cout << " State size : " << insize << std::endl;
			break;
			case 'A' :
				// Set action number !
				outsize = atoi(argv[optind]);
				std::cout << " Action number : " << outsize << std::endl;
			break;
			case 'L' :
				// Set Seed for random values
				L = argv[optind];
				std::cout << " Load knowledge from : " << L << std::endl;
			break;
			case 'b' :
				// Set Seed for random values
				fixedSeed = atoi(argv[optind]);
				std::cout << " Seed set : " << fixedSeed << std::endl;
			break;
			case 'x' : // set eXperience ID
				x = atoi(argv[optind]);
				std::cout << " Experience : " << x << std::endl;
			break;
			case 'd' : // set experience Duration
				d = atoi(argv[optind]);
				std::cout << " Duration : " << d << std::endl;
			break;
			case 'D' : // set experience Duration
				D = argv[optind];
				std::cout << " time unit : " << D << std::endl;
			break;
			case 't' : // set Tau (Temperature of softmax)
				t = atof(argv[optind]);
				std::cout << " Softmax Temperature : " << t << std::endl;
			break;
			case 'a' : // set Alpha (learning rate)
				a = atof(argv[optind]);
				std::cout << " Learning rate : " << a << std::endl;
			break;
			case 'g' : // set Gamma (discount factor)
				g = atof(argv[optind]);
				std::cout << " Discount factor : " << g << std::endl;
			break;
		//	case 's' : // set elementof memory duration
		///		s = atof(argv[optind]);
		//		std::cout << " Max duration of an internal state : " << s << std::endl;
		//	break;
			case 'h' : // show Help option
				std::cout   << " Help : " << std::endl
							<< "Usage : rosrun BP_experiment qlneural <-h (help) | options>" << std::endl
							<< std::endl
							<< "-a \t\t : learning rate of Qlearning (alpha)" << std::endl
							<< "-g \t\t : discount factor of Qlearning (gamma)" << std::endl
							<< "-t \t\t : temperature of Softmax decision rule (tau)" << std::endl
							<< std::endl
							<< "-d \t\t : duration of a run (in iterations ; 0 means no duration limit)" << std::endl
							<< "-x \t\t : eXperience ID for logging files" << std::endl
							<< std::endl
					//		<< "-s \t\t : max duration of an internal state, in seconds. If no external perception ... " << std::endl
						//	<< "-v \t\t : range of variation of space between blocks on the belt" << std::endl
							<< std::endl
							<< "-h \t\t : print this text and exit" << std::endl;
							exit(EXIT_SUCCESS);
			break;
			default :
				std::cout   << "unknown option" << std::endl
							<< "This node expects some arguments, use " << std::endl
							<< "\t rosrun BP_experiment modelfree -h" << std::endl
							<< "to get help." << std::endl;
							exit(EXIT_FAILURE);
			break;
		}
	}
	
	srand( ((fixedSeed == -1) ? time(NULL) : fixedSeed) );

	// Testing if seed use is OK :
//	std::cout << rand() << " " << rand() << " " << rand() << " " << rand() << " " << rand() << " " << rand() << " " << rand() <<     " " << rand() << " " << rand() << " " << rand() << " " <<std::endl;

	// Instantiate a NavExpManager Expert
	NavExpManager manager(nh, x, d, D, L);
	// Start node
	manager.run();


	std::cout << "ending...." << std::endl;

	return EXIT_SUCCESS;
}
