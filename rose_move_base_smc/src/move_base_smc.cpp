/***********************************************************************************
* Copyright: Rose B.V. (2014)
*
* Revision History:
*	Author: Okke Hendriks
*	Date  : 2014/05/05
* 		- File created.
*
* Description:
*	Class creating a smc wrapper interface for move_base.
*  	This is needed in order to ensure feedback is from the drive controller and Platform controller
* 	is taken into account.
* 
***********************************************************************************/

#include "rose_move_base_smc/move_base_smc.hpp"

MoveBaseSMC::MoveBaseSMC(ros::NodeHandle n, string name)
	: n_(n)
	, name_(name)
	, simple_nav_goal_(false)
	, sh_platform_controller_alarm_(SharedVariable<bool>("platform_controller_alarm"))
	, sh_bumper_pressed_(SharedVariable<bool>("bumper_pressed"))
	, sh_emergency_(SharedVariable<bool>("emergency"))
	, velocity_watchdog_("move_base_smc_velocity_watchdog", VELOCITY_TIMEOUT, boost::bind(&MoveBaseSMC::CB_cancelAllMovements, this))
{
	// Initialize SMC
	smc_ = new SMC(n_, name_, boost::bind(&MoveBaseSMC::CB_moveBaseGoalReceived, this, _1, _2));

    smc_->addClient<move_base_msgs::MoveBaseAction>("move_base", 
		boost::bind(&MoveBaseSMC::CB_moveBaseSuccess, this, _1, _2),
		boost::bind(&MoveBaseSMC::CB_moveBaseFail, this, _1, _2),
		NULL, 
		boost::bind(&MoveBaseSMC::CB_moveBaseFeedback, this, _1));

    smc_->addClient<rose_base_msgs::cmd_velocityAction>("drive_controller",
    	boost::bind(&MoveBaseSMC::CB_driveControllerSuccess, this, _1, _2),
		boost::bind(&MoveBaseSMC::CB_driveControllerFail, this, _1, _2),
		NULL, 
		boost::bind(&MoveBaseSMC::CB_driveControllerFeedback, this, _1));

    smc_->addClient<rose_relative_positioning::relative_positioningAction>("relative_positioning",
    	boost::bind(&MoveBaseSMC::CB_relative_positioningSuccess, this, _1, _2),
		boost::bind(&MoveBaseSMC::CB_relative_positioningFail, this, _1, _2),
		NULL, 
		boost::bind(&MoveBaseSMC::CB_relative_positioningFeedback, this, _1));

    smc_->startServer();

    // Initialize subscribers
    move_base_simple_goal_sub_ 	= n_.subscribe("/move_base_smc/simple_goal", 1, &MoveBaseSMC::CB_simpleGoal, this);
    cmd_vel_sub_		 		= n_.subscribe("/cmd_vel", 1, &MoveBaseSMC::CB_commandVelocity, this);
    manual_cmd_vel_sub_			= n_.subscribe("/manual_cmd_vel", 1, &MoveBaseSMC::CB_manualCommandVelocity, this);

    // Initialize publishers
    drivetrain_state_pub_     	= n_.advertise<std_msgs::Int32>("/move_base_smc/drivetrain_state", 1);

	// Monitor the low-level platform controller alarm state
	sh_platform_controller_alarm_.connect(ros::Duration(0.1));
	sh_platform_controller_alarm_.registerChangeCallback(boost::bind(&MoveBaseSMC::CB_platform_alarm, this, _1));

	// Monitor the low-level platform controller alarm state //! @todo OH: HACK
	sh_bumper_pressed_.connect(ros::Duration(0.1));
	sh_bumper_pressed_.registerChangeCallback(boost::bind(&MoveBaseSMC::CB_bumper_pressed, this,  _1));

	// Monitor the emergency button state
	sh_emergency_.connect(ros::Duration(0.1));
	sh_emergency_.registerChangeCallback(boost::bind(&MoveBaseSMC::CB_emergency, this,  _1));
}

MoveBaseSMC::~MoveBaseSMC()
{}

void MoveBaseSMC::stop()
{
	ROS_INFO_NAMED(ROS_NAME, "Stopping base.");
	smc_->abort();

	geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.linear.z = 0.0;
    cmd_vel.angular.x = 0.0;
    cmd_vel.angular.y = 0.0;
    cmd_vel.angular.z = 0.0;

	sendDriveControllerGoal(cmd_vel);
}

void MoveBaseSMC::sendGoal(const move_base_msgs::MoveBaseGoal& goal_to_send)
{
	// Forward this goal to either the move_base node or the relative_positioning node depending on the frame
	if(goal_to_send.target_pose.header.frame_id == "map" || goal_to_send.target_pose.header.frame_id == "/map") 		//! @todo OH: make configurable
	{
		ROS_INFO_NAMED(ROS_NAME, "Received goal in '%s' frame, sending goal to move_base node.", goal_to_send.target_pose.header.frame_id.c_str());
		
		// Could be that the previous goal was a relative goal therefore cancel the relative_positioning, if it did not had a goal notthing happens
		smc_->cancelClient("relative_positioning");

		move_base_msgs::MoveBaseGoal move_base_goal;
		move_base_goal.target_pose = goal_to_send.target_pose;
		//! @todo OH: if not issue #580
		smc_->sendGoal<move_base_msgs::MoveBaseAction>(move_base_goal, "move_base");
	}
	else
	{
		ROS_INFO_NAMED(ROS_NAME, "Received goal in '%s' frame, sending goal to relative_positioning node.", goal_to_send.target_pose.header.frame_id.c_str());
		
		// Could be that the previous goal was a relative goal therefore cancel the move_base, if it did not had a goal notthing happens
		smc_->cancelClient("move_base");

		rose_relative_positioning::relative_positioningGoal relative_positioning_goal;
		relative_positioning_goal.reference_pose = goal_to_send.target_pose;
		//! @todo OH: if not issue #580
		smc_->sendGoal<rose_relative_positioning::relative_positioningAction>(relative_positioning_goal, "relative_positioning");
	}
}

void MoveBaseSMC::sendDriveControllerGoal(const geometry_msgs::Twist& cmd_vel)
{
	ros::Time start_time = ros::Time::now();

	rose_base_msgs::cmd_velocityGoal goal;
    goal.cmd_vel = cmd_vel;
    //! @todo OH: if not issue #580
    smc_->sendGoal<rose_base_msgs::cmd_velocityAction>(goal, "drive_controller", 1.0/25.0);
}

void MoveBaseSMC::CB_moveBaseGoalReceived(const move_base_msgs::MoveBaseGoalConstPtr& goal, SMC* smc)
{
	ROS_DEBUG_NAMED(ROS_NAME_SMC, "CB_moveBaseGoalReceived");

	// Keep track of the fact that this has NOT been a simple goal.
	simple_nav_goal_ = false;
	sendGoal(*goal);
}

void MoveBaseSMC::CB_moveBaseSuccess(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& client_result)
{
	ROS_DEBUG_NAMED(ROS_NAME_SMC, "CB_moveBaseSuccess");
	if(!simple_nav_goal_)
	{
		// Set server goal succesfull, give clients some time to finish
		smc_->sendServerResult(true, *client_result, ros::Duration(SERVER_RESULT_CLIENTS_TIMEOUT)); //! @todo OH: Move base smc needs own action with usefull result
	}
}

void MoveBaseSMC::CB_moveBaseFail(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& client_result)
{
	ROS_DEBUG_NAMED(ROS_NAME_SMC, "CB_moveBaseFail");
	
	if(!simple_nav_goal_)
	{
		// Set server goal failed, give clients some time to finish
		smc_->sendServerResult(false, *client_result, ros::Duration(SERVER_RESULT_CLIENTS_TIMEOUT)); //! @todo OH: Move base smc needs own action with usefull result
	}
}

void MoveBaseSMC::CB_moveBaseFeedback(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback)
{
	ROS_DEBUG_NAMED(ROS_NAME_SMC, "CB_moveBaseFeedback");
	
	if(!simple_nav_goal_)
		smc_->sendServerFeedback(*feedback);	//! @todo OH: Move base smc needs own action with usefull result
}

void MoveBaseSMC::CB_simpleGoal(const geometry_msgs::PoseStampedConstPtr& pose)
{
	// Create a goal with the requested target pose
	move_base_msgs::MoveBaseGoal goal;
	goal.target_pose = *pose;

	// Keep track of the fact that this has been a simple goal (no actionlib client on the other side thus we cannot send feedback or results)
	simple_nav_goal_ = true;
	sendGoal(goal);
}

void MoveBaseSMC::CB_commandVelocity(const geometry_msgs::Twist::ConstPtr& cmd_vel)
{
	ROS_DEBUG_NAMED(ROS_NAME, "Received a command velocity on topic /cmd_vel, forwarding via SMC interface to the drive controller.");

	sendDriveControllerGoal(*cmd_vel);
}

void MoveBaseSMC::CB_manualCommandVelocity(const geometry_msgs::TwistStamped::ConstPtr& cmd_vel)	 //! @todo OH: command from moveto operation should have its own action message that combines the goal of a auto nav and a cmd_vel of manual
{
	// A manual command velocity overrides the automatic ones.
	ROS_DEBUG_NAMED(ROS_NAME, "Received a MANUAL command velocity on topic /manual_cmd_vel, forwarding via SMC interface to the drive controller.");

	if(smc_->isClientBusy("move_base") || smc_->isClientBusy("relative_positioning") ) //! @todo race condition?
	{
		ROS_INFO_NAMED(ROS_NAME, "Manual command velocity received, canceling all goals.");
		smc_->cancelAllClients();
	}

	//! @todo OH: waitForFail CB stuff (add to SMC?)

	// Cancel the goal that the server had, if it had one
	move_base_msgs::MoveBaseResult server_result;
	smc_->sendServerResult(false, server_result, ros::Duration(SERVER_RESULT_CLIENTS_TIMEOUT)); 

    if ( not velocity_watchdog_.reset(cmd_vel->header) )
    {
        ROS_WARN("Received velocity command is too old.");
        return;
    }
	// Send the manual command veloctiy that was revceived
	sendDriveControllerGoal(cmd_vel->twist);
}

void MoveBaseSMC::CB_driveControllerSuccess(const actionlib::SimpleClientGoalState& state, const rose_base_msgs::cmd_velocityResultConstPtr& client_result)
{
	ROS_DEBUG_NAMED(ROS_NAME, "Command velocity successfully forwarded from /cmd_vel to drive controller SMC interface.");
}

void MoveBaseSMC::CB_driveControllerFail(const actionlib::SimpleClientGoalState& state, const rose_base_msgs::cmd_velocityResultConstPtr& client_result)
{
	ROS_WARN_NAMED(ROS_NAME, "Error while forwarding command velocity from /cmd_vel to drive controller.");

	// Publish the return code for use in the local planner? //! @todo OH: remove?
	std_msgs::Int32 return_code; 
	return_code.data = client_result->return_code;	
	drivetrain_state_pub_.publish(return_code);

	// Cancel the goal that the server had
	move_base_msgs::MoveBaseResult server_result;
	smc_->sendServerResult(false, server_result); 
}

void MoveBaseSMC::CB_driveControllerFeedback(const rose_base_msgs::cmd_velocityFeedbackConstPtr& feedback)
{
	ROS_DEBUG_NAMED(ROS_NAME, "CB_driveControllerFeedback");

	//! @todo OH: command from moveto operation should have its own action message that combines the feedback/(goal?) from move_base and the drive line.
}


void MoveBaseSMC::CB_relative_positioningSuccess(const actionlib::SimpleClientGoalState& state, const rose_relative_positioning::relative_positioningResultConstPtr& client_result)
{
	ROS_DEBUG_NAMED(ROS_NAME, "CB_relative_positioningSuccess");
	ROS_DEBUG_NAMED(ROS_NAME, "Succesfully positioned robot using relative positioning.");

	move_base_msgs::MoveBaseResult server_result;
	smc_->sendServerResult(true, server_result, ros::Duration(SERVER_RESULT_CLIENTS_TIMEOUT));	//! @todo OH: Move base smc needs own action with usefull result
}

void MoveBaseSMC::CB_relative_positioningFail(const actionlib::SimpleClientGoalState& state, const rose_relative_positioning::relative_positioningResultConstPtr& client_result)
{
	ROS_DEBUG_NAMED(ROS_NAME, "CB_relative_positioningFail");
	ROS_WARN_NAMED(ROS_NAME, "Error while trying to position robot using relative positioning.");

	move_base_msgs::MoveBaseResult server_result;
	smc_->sendServerResult(false, server_result, ros::Duration(SERVER_RESULT_CLIENTS_TIMEOUT));	//! @todo OH: Move base smc needs own action with usefull result
}

void MoveBaseSMC::CB_relative_positioningFeedback(const rose_relative_positioning::relative_positioningFeedbackConstPtr& feedback)
{
	ROS_DEBUG_NAMED(ROS_NAME, "CB_relative_positioningFeedback");
	sendDriveControllerGoal(feedback->cmd_vel);
}

void MoveBaseSMC::CB_platform_alarm(const bool& new_value)
{
	ROS_WARN_NAMED(ROS_NAME, "CB_platform_alarm");
	if(new_value == true)
	{
		stop();
		operator_gui.warn("Gestopt met rijden vanwege alarm platform aansturing.");
	}
}

void MoveBaseSMC::CB_bumper_pressed(const bool& new_value)
{
	ROS_WARN_NAMED(ROS_NAME, "CB_bumper_pressed");
	if(new_value == true)
	{
		operator_gui.warn("Ingedrukte bumper.");
	}
}

void MoveBaseSMC::CB_emergency(const bool& new_value)
{
	ROS_WARN_NAMED(ROS_NAME, "CB_emergency");
	if(new_value == true)
	{
		stop();
		operator_gui.warn("Gestopt met rijden vanwege nood toestand.");
	}
}

void MoveBaseSMC::CB_cancelAllMovements()
{
    velocity_watchdog_.stop();

	// A manual command velocity overrides the automatic ones.
	ROS_DEBUG_NAMED(ROS_NAME, "Watchdog timeout: Canceling movement.");

	if(smc_->isClientBusy("move_base") || smc_->isClientBusy("relative_positioning") ) //! @todo race condition?
	{
		ROS_INFO_NAMED(ROS_NAME, "Manual command velocity received, canceling all goals.");
		smc_->cancelAllClients();
	}

	//! @todo OH: waitForFail CB stuff (add to SMC?)

	// Cancel the goal that the server had, if it had one
	move_base_msgs::MoveBaseResult server_result;
	smc_->sendServerResult(false, server_result, ros::Duration(SERVER_RESULT_CLIENTS_TIMEOUT)); 

	geometry_msgs::Twist cmd_vel; // Empty twist

	// Send the manual command veloctiy that was received
	sendDriveControllerGoal(cmd_vel);
}
