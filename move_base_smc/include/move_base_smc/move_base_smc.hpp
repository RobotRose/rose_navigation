#ifndef MOVE_BASE_SMC_HPP
#define MOVE_BASE_SMC_HPP

#include <std_msgs/Int32.h> 
#include <geometry_msgs/Twist.h> 
#include <geometry_msgs/PoseStamped.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <move_base_msgs/MoveBaseActionFeedback.h>

#include "rose_common/common.hpp"
#include "server_multiple_client/server_multiple_client.hpp"
#include "operator_messaging/operator_messaging.hpp"

#include "shared_variables/shared_variable.hpp"

#include "rose_base_msgs/cmd_velocityAction.h"
#include "rose_base_msgs/cmd_velocityActionGoal.h"
#include "rose_base_msgs/cmd_velocityActionResult.h"
#include "rose_base_msgs/cmd_velocityActionFeedback.h"

#include "relative_positioning/relative_positioning.hpp"
#include "relative_positioning/relative_positioningAction.h"
#include "relative_positioning/relative_positioningActionGoal.h"
#include "relative_positioning/relative_positioningActionResult.h"
#include "relative_positioning/relative_positioningActionFeedback.h"

#define SERVER_RESULT_CLIENTS_TIMEOUT 	0.5

using namespace std; 
using namespace shared_variables;

class MoveBaseSMC
{
  protected:
  	typedef ServerMultipleClient<move_base_msgs::MoveBaseAction> SMC;

  public:
	MoveBaseSMC(ros::NodeHandle n, string name);
	~MoveBaseSMC();

  private:
  	void sendGoal(const move_base_msgs::MoveBaseGoal& goal);
  	void sendDriveControllerGoal(const geometry_msgs::Twist& cmd_vel);

  	void stop();

  	void CB_moveBaseGoalReceived(const move_base_msgs::MoveBaseGoalConstPtr& goal, SMC* smc);

	void CB_moveBaseSuccess(	const actionlib::SimpleClientGoalState& state, 
								const move_base_msgs::MoveBaseResultConstPtr& client_result);
	
	void CB_moveBaseFail(		const actionlib::SimpleClientGoalState& state, 
								const move_base_msgs::MoveBaseResultConstPtr& client_result);

	void CB_moveBaseFeedback(	const move_base_msgs::MoveBaseFeedbackConstPtr& feedback);

	void CB_simpleGoal(			const geometry_msgs::PoseStampedConstPtr& pose);

	void CB_commandVelocity(		const geometry_msgs::Twist::ConstPtr& cmd_vel);

	void CB_manualCommandVelocity(	const geometry_msgs::Twist::ConstPtr& cmd_vel);

	void CB_driveControllerSuccess(	const actionlib::SimpleClientGoalState& state, 
									const rose_base_msgs::cmd_velocityResultConstPtr& client_result);
	
	void CB_driveControllerFail(	const actionlib::SimpleClientGoalState& state, 
									const rose_base_msgs::cmd_velocityResultConstPtr& client_result);
	
	void CB_driveControllerFeedback(const rose_base_msgs::cmd_velocityFeedbackConstPtr& feedback);

	void CB_relative_positioningSuccess(const actionlib::SimpleClientGoalState& state, const relative_positioning::relative_positioningResultConstPtr& client_result);

	void CB_relative_positioningFail(const actionlib::SimpleClientGoalState& state, const relative_positioning::relative_positioningResultConstPtr& client_result);

	void CB_relative_positioningFeedback(const relative_positioning::relative_positioningFeedbackConstPtr& feedback);

	void CB_platform_alarm(const bool& new_value);
	void CB_bumper_pressed(const bool& new_value);
	void CB_emergency(const bool& new_value);

	// Variables
  	ros::NodeHandle n_;
  	string 			name_;
  	SMC* 			smc_;
  	bool 			simple_nav_goal_;

  	// Subscribers
  	ros::Subscriber move_base_simple_goal_sub_;
  	ros::Subscriber cmd_vel_sub_;
  	ros::Subscriber manual_cmd_vel_sub_;

  	// Publishers
  	ros::Publisher 	drivetrain_state_pub_;

    SharedVariable<bool>            sh_platform_controller_alarm_;
    SharedVariable<bool>            sh_bumper_pressed_;
    SharedVariable<bool>            sh_emergency_;
    OperatorMessaging      			operator_gui;
};

#endif // MOVE_BASE_SMC_HPP
