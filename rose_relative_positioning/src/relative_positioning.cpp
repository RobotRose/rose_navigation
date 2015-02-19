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

#include "rose_relative_positioning/relative_positioning.hpp"

RelativePositioning::RelativePositioning(ros::NodeHandle n, std::string name)
    : n_(n)
    , name_(name)
    , platform_(GenericPlatform("rose20"))
    , sh_bumper_pressed_(SharedVariable<bool>("bumper_pressed"))
{
    // Register target pose pulisher
    target_pose_pub_ = n_.advertise<geometry_msgs::PoseStamped>("target_pose", 0);

    // Initialize SMC
    smc_ = new SMC(n_, name_, boost::bind(&RelativePositioning::CB_relativePositioningGoalReceived, this, _1, _2));

    smc_->startServer();

    // Monitor the low-level platform controller alarm state //! @todo OH: HACK
    sh_bumper_pressed_.connect(ros::Duration(0.1));
}

RelativePositioning::~RelativePositioning()
{

}

void RelativePositioning::CB_relativePositioningGoalReceived(const rose_relative_positioning::relative_positioningGoalConstPtr& goal, SMC* smc)
{
    rose_relative_positioning::relative_positioningResult server_result;

    bool converged = calculateCommandVelocity(goal->reference_pose);
    
    if(converged)
    {
        server_result.return_code = SUCCESS;
        server_result.error_code  = 0;
    }
    else
    {
        server_result.return_code = RELATIVE_POSITIONING_NOT_CONVERGED_ERROR;
        server_result.error_code  = 0;
    }

    smc_->sendServerResult(converged, server_result);
}


bool RelativePositioning::calculateCommandVelocity(const geometry_msgs::PoseStamped reference_pose)   
{
    ros::Rate r(25);

    rose_relative_positioning::relative_positioningFeedback server_feedback;

    geometry_msgs::PoseStamped cur_pose;
    geometry_msgs::PoseStamped target_pose = reference_pose;    

    if ( not rose_transformations::transformToFrameNow(tf_listener_, "odom", target_pose))    //! @todo OH: Make configurable
    {
        ROS_WARN_NAMED(ROS_NAME, "Could not transform 'odom' frame to '%s' frame.", target_pose.header.frame_id.c_str());
        return false;
    }

    target_pose_pub_.publish(target_pose);
    
    rose_geometry::Point dtranslation;
    rose_geometry::Point prev_dtranslation;
    
    float dyaw          = 0.0;
    bool converged      = false;

    float prev_dyaw     = std::numeric_limits<float>::max();
    prev_dtranslation.x = std::numeric_limits<float>::max();
    prev_dtranslation.y = std::numeric_limits<float>::max();

    geometry_msgs::Twist cmd_vel;
      // Stop if goal is canceled or when converged.
    while ( smc_->hasActiveGoal() && !converged )
    {
        if(sh_bumper_pressed_)
        {
            ROS_WARN_NAMED(ROS_NAME, "Stopping relative positioning because a bumper has been pressed.");
            operator_gui.warn("Positioneren gestopt vanwege ingedrukte bumper.");
            return false;
        }
        
        if(!rose_transformations::getFrameInFrame(tf_listener_, "base_link", "odom", cur_pose))     //! @todo OH: Make configurable
        {
            ROS_WARN_NAMED(ROS_NAME, "Could not retreive base_link frame in odom frame.");
            return false;
        }

        // Calculate the deltas
        dtranslation    = rose_geometry::Point(target_pose.pose.position) - rose_geometry::Point(cur_pose.pose.position);
        dyaw            = rose_geometry::getShortestSignedAngle(tf::getYaw(cur_pose.pose.orientation), tf::getYaw(target_pose.pose.orientation));

        // Rotate the command velocity vector such that it is in the odom frame
        rose_geometry::rotateVect(&dtranslation.x, &dtranslation.y, -tf::getYaw(cur_pose.pose.orientation));

        // First do turn then strafe
        // Higher than treshold, OR still decreasing
        if(std::fabs(dyaw) >= YAW_CONVERGE_ANGLE || std::fabs(dyaw) < std::fabs(prev_dyaw))
        {
            ROS_DEBUG_THROTTLE_NAMED(0.25, ROS_NAME, "Now converging orientation.");
            
            rose_conversions::getRampedVelocity(
                platform_.getMinVelocity("THETA_INPLACE"), 
                platform_.getMaxVelocity("THETA_INPLACE"),
                GOAL_MIN_ANGLE_DIST,
                GOAL_MAX_ANGLE_DIST,
                dyaw,
                cmd_vel.angular.z);            

            // Calculate the resulting velocity based on the delta in the rotation
            cmd_vel.linear.x = 0.0;
            cmd_vel.linear.y = 0.0;
            cmd_vel.linear.z = 0.0;
            cmd_vel.angular.x = 0.0;
            cmd_vel.angular.y = 0.0;
            // cmd_vel.angular.z = dyaw; 

            // Limit the velocity to the maximal theta velocity 
            // limit(-platform_.getMaxVelocity("THETA_INPLACE"), platform_.getMaxVelocity("THETA_INPLACE"), &cmd_vel.angular.z); 
            // if(std::fabs(cmd_vel.angular.z) <= platform_.getMinVelocity("THETA_INPLACE"))
            //     cmd_vel.angular.z = rose_conversions::sgn(cmd_vel.angular.z) * platform_.getMinVelocity("THETA_INPLACE");

          

        }
        // Higher than treshold, OR still decreasing
        else if(std::fabs(dtranslation.x) >= XY_CONVERGE_DIST || 
                std::fabs(dtranslation.y) >= XY_CONVERGE_DIST ||
                std::fabs(dtranslation.x) < std::fabs(prev_dtranslation.x) || 
                std::fabs(dtranslation.y) < std::fabs(prev_dtranslation.y)
                )
        {
            ROS_DEBUG_THROTTLE_NAMED(0.25, ROS_NAME, "Orientation converged, now converging x & y.");

            ROS_DEBUG_THROTTLE_NAMED(0.25, ROS_NAME, "Distance speed: (%f, %f).", dtranslation.x, dtranslation.y);

            rose_geometry::Point xy_speed;

            rose_conversions::getRampedVelocity(
                // platform_.getMinVelocity("XY_ABS"), 
                // platform_.getMaxVelocity("XY_ABS"),
                0.01, 0.10,
                GOAL_MIN_VEL_DIST,
                GOAL_MAX_VEL_DIST,
                dtranslation,
                xy_speed);

            ROS_DEBUG_THROTTLE_NAMED(0.25, ROS_NAME, "Ramped speed: (%f, %f).", cmd_vel.linear.x, cmd_vel.linear.y);
            
            // Calculate the resulting velocity based on the deltas in X and Y
            cmd_vel.linear.x = xy_speed.x;
            cmd_vel.linear.y = xy_speed.y;
            cmd_vel.linear.z = 0.0;
            cmd_vel.angular.x = 0.0;
            cmd_vel.angular.y = 0.0;
            cmd_vel.angular.z = 0.0; 

            // // Limit the command velocity vector to the maximal velocity value
            // rose_geometry::limitVectorLengthXY(&cmd_vel.linear.x, &cmd_vel.linear.y, platform_.getMaxVelocity("XY_ABS"));

            // // Limit the command velocity vector to the minimal velocity value
            // if(rose_geometry::getVectorLengthXY(cmd_vel.linear.x, cmd_vel.linear.y) < platform_.getMinVelocity("XY_ABS"))
            //     rose_geometry::setVectorLengthXY(&cmd_vel.linear.x, &cmd_vel.linear.y, platform_.getMinVelocity("XY_ABS"));
             

        }
        else
            converged = true; 

        // Update values
        prev_dyaw           = std::min(std::fabs(dyaw), prev_dyaw);
        prev_dtranslation.x = std::min(std::fabs(dtranslation.x), prev_dtranslation.x);
        prev_dtranslation.y = std::min(std::fabs(dtranslation.y), prev_dtranslation.y);

        ROS_INFO_THROTTLE_NAMED(0.5, ROS_NAME, "Converging, difference: [%.3fm, %.3fm, %.3fdeg] -> [%.3fm/s, %.3fm/s, %.3fdeg/s]", dtranslation.x, dtranslation.y, dyaw, cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z*(180.0/M_PI));

        // Send the calculated command velocity in the feedback.
        server_feedback.cmd_vel = cmd_vel;
        smc_->sendServerFeedback(server_feedback);

        r.sleep();
    }    

    // Force stop
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.linear.z = 0.0;
    cmd_vel.angular.x = 0.0;
    cmd_vel.angular.y = 0.0;
    cmd_vel.angular.z = 0.0;  

    server_feedback.cmd_vel = cmd_vel;
    smc_->sendServerFeedback(server_feedback);

    if(converged)
        ROS_DEBUG_NAMED(ROS_NAME, "Relative positioning converged!");
    else
        ROS_DEBUG_NAMED(ROS_NAME, "Relative positioning has been canceled."); 
    
    return true;
}

