/***********************************************************************************
* Copyright: Rose B.V. (2014)
*
* Revision History:
*   Author: Okke Hendriks
*   Date  : 2014/08/01
*       - File created.
*
* Description:
*   Given an stamped pose this will generate the required command velocities to 
*   position the base_link in the odometry frame.
* 
***********************************************************************************/

#ifndef RELATIVE_POSITIONING_HPP
#define RELATIVE_POSITIONING_HPP

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>

#include "rose_common/common.hpp"
 
#include "server_multiple_client/server_multiple_client.hpp"
#include "action_result_message.hpp"

#include "shared_variables/shared_variable.hpp"
#include "operator_messaging/operator_messaging.hpp"

#include "rose_generic_platform/generic_platform.hpp"
#include "rose_conversions/conversions.hpp"
#include "rose_geometry/geometry.hpp"
#include "rose_transformations/transformations.hpp"

#include "rose_relative_positioning/relative_positioningAction.h"
#include "rose_relative_positioning/relative_positioningActionGoal.h"
#include "rose_relative_positioning/relative_positioningActionResult.h"
#include "rose_relative_positioning/relative_positioningActionFeedback.h"

#define XY_CONVERGE_DIST        0.005
#define YAW_CONVERGE_ANGLE      0.04

#define GOAL_MIN_ANGLE_DIST 0.05 //< Angle (rads) where the robot should be at lowest speed turning to goal
#define GOAL_MAX_ANGLE_DIST 1.0  //< Angle (rads) where the robot should be at lowest speed turning to goal
#define GOAL_MIN_VEL_DIST   0.05 //< Distance where the robot should be at lowest speed driving to goal
#define GOAL_MAX_VEL_DIST   0.30  //< Distance where the robot should be at lowest speed driving to goal

using namespace shared_variables;   //! @todo OH [LANG]: No using namespaces!

class RelativePositioning
{
  protected:
    typedef ServerMultipleClient<relative_positioning::relative_positioningAction> SMC;

  public:
    RelativePositioning(ros::NodeHandle n, std::string name);
    ~RelativePositioning();

    void CB_relativePositioningGoalReceived(const relative_positioning::relative_positioningGoalConstPtr& goal, SMC* smc);

    bool calculateCommandVelocity(const geometry_msgs::PoseStamped reference_pose);

  private:

    ros::NodeHandle         n_;
    std::string             name_;
    tf::TransformListener   tf_listener_;
    SMC*                    smc_;
    GenericPlatform         platform_;

    // Publishers
    ros::Publisher          target_pose_pub_;

    SharedVariable<bool>    sh_bumper_pressed_;

    OperatorMessaging       operator_gui;
};


#endif //RELATIVE_POSITIONING_HPP
