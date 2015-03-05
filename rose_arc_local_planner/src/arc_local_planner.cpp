#include "rose_arc_local_planner/arc_local_planner.hpp"

namespace rose_navigation{

#define INF_RADIUS_VALUE 		10000.0
#define MIN_RADIUS 				0.0
#define MAX_VEL_X 				0.24
#define MAX_VEL_Y 				0.24
#define MAX_VEL_ABS 			sqrt(MAX_VEL_X*MAX_VEL_X + MAX_VEL_Y*MAX_VEL_Y)
#define MAX_VEL_THETA 			0.3
#define MAX_VEL_THETA_INPLACE	0.4

#define MIN_VEL_ABS 			0.05
#define MIN_VEL_THETA 			0.0001
#define MIN_VEL_THETA_INPLACE 	0.15
#define MAX_ACC_X 				0.4
#define MAX_ACC_Y 				0.4
#define MAX_ACC_THETA 			0.4
#define ROTATE_SAFETY_ANGLE 	0.08
#define ROTATE_ANGLE_THRESHOLD 	0.04
#define STRAFE_SAFETY_DIST		0.05
#define DISTANCE_MAX_VEL 		1.2
#define MIN_ARC_DIST			0.15	
#define MAX_ANGLE_MEASURE 		M_PI*(4.0/4.0)
#define MAX_ARRIVAL_ANGLE 		M_PI*(4.0/4.0)
#define AT_GOAL_DIST 			0.05
#define AT_GOAL_ANGLE 			0.10
#define CMD_VEL_MAF_WINDOW 		1	

//! @todo OH: Use Rose20Platform class


PLUGINLIB_EXPORT_CLASS(rose_navigation::ArcLocalPlanner, nav_core::BaseLocalPlanner);

using std::vector;
using std::string;

using geometry_msgs::Point32;
using geometry_msgs::Pose;
using geometry_msgs::PoseStamped;
using geometry_msgs::Twist;
using base_local_planner::Position2DInt;

ArcLocalPlanner::ArcLocalPlanner()
	: initialized_(false)
	, costmap_ros_(NULL)
	, odom_helper_("odom")
	, state_(DRIVE)
	, prev_state_(DRIVE)
{}

ArcLocalPlanner::ArcLocalPlanner(string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros)
	: initialized_(false)
	, costmap_ros_(NULL)
	, odom_helper_("odom")  
	, state_(DRIVE)
	, prev_state_(DRIVE)
{
	initialize(name, tf, costmap_ros);
}

ArcLocalPlanner::~ArcLocalPlanner()
{}

void ArcLocalPlanner::initialize(string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros)
{
	ROS_DEBUG_NAMED(ROS_NAME, "ArcLocalPlanner::initialize");

	if(!initialized_)
	{
		name_		 = name;
		tf_ 		 = tf;
		costmap_ros_ = costmap_ros;// new costmap_2d::Costmap2DROS("local_costmap", *tf_); 
		costmap_     = costmap_ros_->getCostmap();

		global_frame_ 		= costmap_ros_->getGlobalFrameID();
        robot_base_frame_ 	= costmap_ros_->getBaseFrameID();

        ROS_INFO_NAMED(ROS_NAME, "ArcLocalPlanner: global frame: %s, robot base frame: %s", global_frame_.c_str(), robot_base_frame_.c_str());

        vector<geometry_msgs::Point> footprint_ros = costmap_ros_->getRobotFootprint();
	    footprint_ 			= vector<rose_geometry::Point>(footprint_ros.begin(), footprint_ros.end());

	    // Get a nodehandle 
	    pn_ = ros::NodeHandle("~");

	    // Load ÁLP parameters
    	loadParameters();

		// Setup publishers
	    g_plan_pub_ 		 	= pn_.advertise<nav_msgs::Path>("global_plan", 1);
	    t_plan_pub_ 		 	= pn_.advertise<nav_msgs::Path>("transformed_plan", 1);
	    l_plan_pub_ 		 	= pn_.advertise<nav_msgs::Path>("local_plan", 1);
	    simulation_plan_pub_ 	= pn_.advertise<nav_msgs::Path>("simulation_plan", 1);	
      	rviz_markers_pub_ 		= pn_.advertise<visualization_msgs::MarkerArray>( "debug_marker_array", 0 );
      	rviz_marker_pub_ 		= pn_.advertise<visualization_msgs::Marker>( "debug_marker", 0 );

      	world_model_ = new ArcLocalPlannerCostmapModel(*costmap_);

      	costmap_2d::calculateMinAndMaxDistances(costmap_ros_->getRobotFootprint(), inscribed_radius_, circumscribed_radius_);
      	ROS_INFO_NAMED(ROS_NAME, "Calulated inscribed radius    : %.4fm", inscribed_radius_);
      	ROS_INFO_NAMED(ROS_NAME, "Calulated circumscribed radius: %.4fm", circumscribed_radius_);

      	marker_list_id_ 		= 0;
      	initialized_ 			= true;

      	prev_ranking_ 			= 0.0;
      	prev_index_ 			= 0;

      	distance_squared_prev_ 	= 0.0;
      	angle_difference_prev_ 	= 0.0;

      	cmd_vel_maf_.setWindowSize(CMD_VEL_MAF_WINDOW);

      	begin_ = ros::Time::now();
	}
}

void ArcLocalPlanner::loadParameters()
{
	pn_.param("cost_function_weights/distance",		distance_weight_, 	0.15);
    pn_.param("cost_function_weights/clearance", 	clearance_weight_, 	1.50);
    pn_.param("cost_function_weights/difference", 	difference_weight_,	0.10);
	ROS_INFO_NAMED(ROS_NAME, "Loaded arc local planner parameters");
}

bool ArcLocalPlanner::setPlan(const vector<PoseStamped>& plan)
{
	if (! isInitialized()) {
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }


    if(	not global_plan_.empty() &&
    	global_goal_.pose.position.x == global_plan_.back().pose.position.x &&
    	global_goal_.pose.position.y == global_plan_.back().pose.position.y &&
    	global_goal_.pose.position.z == global_plan_.back().pose.position.z &&
    	global_goal_.pose.orientation.x == global_plan_.back().pose.orientation.x &&
    	global_goal_.pose.orientation.y == global_plan_.back().pose.orientation.y &&
    	global_goal_.pose.orientation.z == global_plan_.back().pose.orientation.z &&
    	global_goal_.pose.orientation.w == global_plan_.back().pose.orientation.w &&
        (ros::Time::now() - global_plan_.back().header.stamp).toSec()  < 5.0			//! @todo OH: Magic number, make configurable (same as update rate global planner?)
    	)
    {
    	ROS_INFO_NAMED(ROS_NAME, "time: %.2f", (ros::Time::now() - global_plan_.back().header.stamp).toSec() );
    	ROS_INFO_NAMED(ROS_NAME, "Updated global plan.", plan.size());	
    }
    else
    {
    	ROS_INFO_NAMED(ROS_NAME, "New global plan recevied, the plan contains %lu points. Resetting the local planner state machine.", plan.size());	
        prev_state_ 	= DRIVE; 
    	state_ 		 	= DRIVE;
    }	

	// Reset and copy the global plan
    global_plan_.clear();
    global_plan_ = plan;

    // Store current global goal
    global_goal_ 						= global_plan_.back();
    global_plan_.back().header.stamp 	= ros::Time::now();

    base_local_planner::publishPlan(global_plan_, g_plan_pub_);
    start_index_ 	= 0;

	return true;
}

bool ArcLocalPlanner::computeVelocityCommands(Twist& cmd_vel)
{	
	ros::Time end = ros::Time::now();
	ros::Duration d = end - begin_; 
	ROS_INFO_NAMED(ROS_NAME, "ALP rate: %.2f", 1.0/d.toSec());	
	begin_ = ros::Time::now();

	Twist new_cmd_vel;
	ROS_DEBUG_NAMED(ROS_NAME, "ArcLocalPlanner::computeVelocityCommands");

	if (! isInitialized()) 
	{
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }

    // Update the global pose and velocity of the robot
    tf::Stamped<tf::Pose> 		global_pose_tf;
    if(!costmap_ros_->getRobotPose(global_pose_tf))
    {
    	ROS_WARN_NAMED(ROS_NAME, "Could not get robot pose from costmap, aborting local planner.");
       	return false;
    }

   	tf::poseStampedTFToMsg (global_pose_tf, global_pose_);
   	float global_yaw = tf::getYaw(global_pose_.pose.orientation);

   	tf::Stamped<tf::Pose> local_vel_tf;
    odom_helper_.getRobotVel(local_vel_tf);
    local_vel_.linear.x  = local_vel_tf.getOrigin().getX();
    local_vel_.linear.x  = local_vel_tf.getOrigin().getY();
    local_vel_.angular.z = tf::getYaw(local_vel_tf.getRotation());
   
    //Transform the global plan to our frame
    ros::Time now = ros::Time::now();
    tf_->waitForTransform("/map", "/base_link", now, ros::Duration(2.0));
    if (!base_local_planner::transformGlobalPlan(*tf_, global_plan_, global_pose_tf, *costmap_, global_frame_, transformed_plan_)) 
    {
      	ROS_WARN_NAMED(ROS_NAME, "Could not transform the global plan to the frame of the controller");
      	return false;
    }	

    if(rose_geometry::distanceXYsquared(transformed_plan_.back(), global_pose_) < AT_GOAL_DIST*AT_GOAL_DIST*2.0) //! @todo magic number
    {
    	auto goal_point = global_plan_.back();
    	global_plan_.clear();
    	global_plan_.push_back(goal_point);
    }

   	if(transformed_plan_.size() >= 1)
   	{
	    rose_geometry::Point robot_pos  		= global_pose_.pose.position;
	    rose_geometry::Point first_path_point 	= transformed_plan_.front().pose.position;

	    // Get current footprint cells
	    Eigen::Vector3f cur_pos(robot_pos.x, robot_pos.y, tf::getYaw(global_pose_.pose.orientation));
    	cur_footprint_cells_ = footprint_helper_.getFootprintCells(cur_pos, costmap_ros_->getRobotFootprint(), *costmap_, true);

		// Make copy of footprint
		transformed_footprint_ = footprint_;
		// Rotate footprint around origin
		rotatePointsAroundOrigin(global_yaw, transformed_footprint_);
		// Translate footprint
		translatePoints(robot_pos.x, robot_pos.y, transformed_footprint_); 		

		publishPolygon(transformed_footprint_, "");

    	// Add current footprint cells to a xy-multimap
	    for(const auto& cell : cur_footprint_cells_)
	        footprint_cells_multimap_[std::make_pair(cell.x, cell.y)] = costmap_->getCost(cell.x, cell.y);

	    VelCalcResult state_result = FINISHED; // Failed, Busy, Finished

	    // ROS_INFO_NAMED(ROS_NAME, "State: %d", state_);
	    // ROS_DEBUG_NAMED(ROS_NAME, "prev_state_ is: %d", prev_state_);

	    ROS_WARN_ONCE_NAMED(ROS_NAME, "Remember to check if arc center is inside base, polygon is not correct at that moment! Use the new Footprint Collision Checker! TODO OH.");

	    found_valid_cmd_vel_ = false;	    
	    do {
	    	StatemachineState new_state = DRIVE;
	    	//state_ = STRAFE_TO_PATH;
	    	// prev_state_ = DRIVE;
	    	
	    	// Reset the command velocity moving average filter if switching mode.
	    	if(prev_state_ != state_)
	    	{
	    		// Reset the cmd_vel moving average filter
	    		cmd_vel_maf_.reset();
	    	}

		    switch(state_)
		    {
		    	case DRIVE:
		    		// Drive towards point over an arc or straight line
				    ROS_INFO_THROTTLE_NAMED(ALP_INFO_THROTTLE, ROS_NAME, "ALP in DRIVE state.");

				    if(prev_state_ != state_)
			    	{
			    		// Store start strafe position
			    		state_start_pose_ 	= global_pose_.pose;

			    		// Final goal is the back of the plan
			    		state_target_pose_ = transformed_plan_.back().pose;
			    	}
			    	
				    state_result = calculateDriveVelocityCommand(new_cmd_vel);
				    
				    if(state_result == FAILED)
			    	{
			    		// Failed, goto next action type
			    		ROS_INFO_NAMED(ROS_NAME, "DRIVE failed, goto AIM_AT_PATH");
			    		new_state 				= AIM_AT_PATH;	
			    		found_valid_cmd_vel_ 	= false;	
			    	}
			    	else if(state_result == BUSY)
			    	{
			    		// Busy, continue
			    		ROS_INFO_NAMED(ROS_NAME, "DRIVE Busy, goto DRIVE");
			    		new_state 				= DRIVE;
			    		found_valid_cmd_vel_ 	= true;
			    	}
			    	else if(state_result == FINISHED)
			    	{
			    		// Finished
			    		ROS_INFO_NAMED(ROS_NAME, "DRIVE Finished, end!");
				    	new_state 				= DRIVE;
				    	found_valid_cmd_vel_ 	= true;
			    	}

				    break;

			    case AIM_AT_PATH:
			    	// Rotate to path
			    	ROS_INFO_THROTTLE_NAMED(ALP_INFO_THROTTLE, ROS_NAME, "ALP in AIM_AT_PATH state.");

			    	if(prev_state_ != state_)
			    	{
			    		// Store start strafe position
			    		state_start_pose_ 	= global_pose_.pose;

			    		// Aim at the path 20 points ahead
			    		state_target_pose_ = getAimAtPathPose(global_pose_.pose, transformed_plan_, 50);
			    		
			    		// Get the direction of the path 20 points ahead
			    		//state_target_pose_ = getPathDirectionPose(global_pose_.pose, transformed_plan_, 20);
			    	}

			    	// 0 -> Failed
			    	// 1 -> Busy
			    	// 2 -> Finished
			    	state_result = calculateRotateVelocityCommand(new_cmd_vel);
			    	if(state_result == FAILED)
			    	{
			    		// Failed, goto next action type
			    		ROS_INFO_NAMED(ROS_NAME, "AIM_AT_PATH failed, goto ALIGN");
			    		new_state 				= ALIGN;	
			    		found_valid_cmd_vel_ 	= true;	
			    	}
			    	else if(state_result == BUSY)
			    	{
			    		// Busy, continue
			    		ROS_INFO_NAMED(ROS_NAME, "AIM_AT_PATH Busy, goto AIM_AT_PATH");
			    		new_state 				= AIM_AT_PATH;
			    		found_valid_cmd_vel_ 	= true;
			    	}
			    	else if(state_result == FINISHED)
			    	{
			    		// Finished
			    		ROS_INFO_NAMED(ROS_NAME, "AIM_AT_PATH Finished, goto DRIVE");
				    	new_state 				= DRIVE;
				    	found_valid_cmd_vel_ 	= true;
			    	}

			    	break;

			    case ALIGN:
			    	// Strafe to the point a bit 'back' of the direction of the path a number of points ahead
			    	ROS_INFO_THROTTLE_NAMED(ALP_INFO_THROTTLE, ROS_NAME, "ALP in ALIGN state.");

			    	if(prev_state_ != state_)
			    	{
			    		// Store start strafe position
			    		state_start_pose_ 	= global_pose_.pose;

			    		// Get the direction of the path 30 points ahead
			    		state_target_pose_ = getAlignPose(global_pose_.pose, transformed_plan_, 30, 40, 0.5);
			    	}

			    	// 0 -> Failed
			    	// 1 -> Busy
			    	// 2 -> Finished
			    	state_result = calculateStrafeVelocityCommand(new_cmd_vel);
			    	if(state_result == FAILED)
			    	{
			    		// Failed, goto next action type
			    		ROS_INFO_NAMED(ROS_NAME, "ALIGN failed, goto STRAFE_TO_PATH");
			    		new_state 				= STRAFE_TO_PATH;	
			    		found_valid_cmd_vel_ 	= true;	
			    	}
			    	else if(state_result == BUSY)
			    	{
			    		// Busy, continue
			    		ROS_INFO_NAMED(ROS_NAME, "ALIGN Busy, goto ALIGN");
			    		new_state 				= ALIGN;
			    		found_valid_cmd_vel_ 	= true;
			    	}
			    	else if(state_result == FINISHED)
			    	{
			    		// Finished
			    		ROS_INFO_NAMED(ROS_NAME, "ALIGN Finished, goto DRIVE");
				    	new_state 				= DRIVE;
				    	found_valid_cmd_vel_ 	= true;
			    	}

			    	break;

			    case STRAFE_TO_PATH:
			    	// Strafe to path
			    	ROS_INFO_THROTTLE_NAMED(ALP_INFO_THROTTLE, ROS_NAME, "ALP in STRAFE_TO_PATH state.");

			    	// Only decide on target at start of strafe action
			    	if(prev_state_ != state_)
			    	{
			    		// Store start strafe position
			    		state_start_pose_ 	= global_pose_.pose;
			    		state_target_pose_	= transformed_plan_.at(getClosestWaypointIndex(global_pose_.pose, transformed_plan_, 50)).pose; //! @todo magic number
			    		limitMaximalStrafeDistance(state_target_pose_);
		    		}

			    	state_result = calculateStrafeVelocityCommand(new_cmd_vel);

		    		// 0 -> Failed
			    	// 1 -> Busy
			    	// 2 -> Finished
			    	if(state_result == FAILED)
			    	{
			    		ROS_INFO_NAMED(ROS_NAME, "STRAFE_TO_PATH Failed, goto STRAFE_TO_OPEN_AREA");
				    	new_state 				= STRAFE_TO_OPEN_AREA;
			    	}
			    	else if(state_result == BUSY)
			    	{
			    		ROS_INFO_NAMED(ROS_NAME, "STRAFE_TO_PATH Busy, goto STRAFE_TO_PATH");
			    		new_state 				= STRAFE_TO_PATH;
			    		found_valid_cmd_vel_ 	= true;
			    	}
			    	else if(state_result == FINISHED)
			    	{		    		
			    		ROS_INFO_NAMED(ROS_NAME, "STRAFE_TO_PATH Finished, goto DRIVE");
						new_state 				= DRIVE;
			    		found_valid_cmd_vel_ 	= true;
			    	}

			    	break;	

			    case STRAFE_TO_OPEN_AREA:		    	
			    	// Strafe
			    	ROS_INFO_THROTTLE_NAMED(ALP_INFO_THROTTLE, ROS_NAME, "ALP in STRAFE_TO_OPEN_AREA state.");

			    	if(prev_state_ != state_)
			    	{
		    			state_start_pose_ 	= global_pose_.pose;
			    		state_target_pose_	= findStrafeTarget(transformed_footprint_, 0.25);		//! @todo magic number
			    		limitMaximalStrafeDistance(state_target_pose_);
			    	}

			    	// 0 -> Failed
			    	// 1 -> Busy
			    	// 2 -> Finished
				    state_result = calculateStrafeVelocityCommand(new_cmd_vel);
				    
			    	if(state_result == FAILED)
			    	{
			    		ROS_INFO_NAMED(ROS_NAME, "STRAFE_TO_OPEN_AREA Failed, return false!");
				    	new_state = DRIVE;

				    	return false;
			    	}
			    	else if(state_result == BUSY)
			    	{
			    		ROS_INFO_NAMED(ROS_NAME, "STRAFE_TO_OPEN_AREA Busy, goto STRAFE_TO_OPEN_AREA");
			    		new_state 			 = STRAFE_TO_OPEN_AREA;
			    		found_valid_cmd_vel_ = true;
			    	}
			    	else if(state_result == FINISHED)
			    	{		    		
			    		ROS_INFO_NAMED(ROS_NAME, "STRAFE_TO_OPEN_AREA Finished, goto DRIVE");
				    	new_state 				= DRIVE;
			    		found_valid_cmd_vel_ 	= true;
			    	}

			    	break;	

			    default:
			    	ROS_ERROR_NAMED(ROS_NAME, "Invalid statemachine state");
			    	break;
			};

			prev_state_ = state_;
			state_ 		= new_state;

		} while(!found_valid_cmd_vel_);
	}
	else
	{
		ROS_ERROR_NAMED(ROS_NAME, "Global path empty?");
		new_cmd_vel.linear.x  = 0.0;
	    new_cmd_vel.linear.y  = 0.0;
	    new_cmd_vel.linear.z  = 0.0;
	    new_cmd_vel.angular.x = 0.0;
	    new_cmd_vel.angular.y = 0.0;
	    new_cmd_vel.angular.z = 0.0;

	    // Force the cmd_vel to zero.
	    cmd_vel = new_cmd_vel;

	    return false;
	}

	// When stopping clear moving average filter to ensure quick stop
	if( new_cmd_vel.linear.x == 0.0 &&
		new_cmd_vel.linear.y == 0.0 &&
		new_cmd_vel.linear.z == 0.0 &&
		new_cmd_vel.angular.x == 0.0 &&
		new_cmd_vel.angular.y == 0.0 &&
		new_cmd_vel.angular.z == 0.0)
	{
	    // Reset the cmd_vel moving average filter
	    cmd_vel_maf_.reset();
	}

	// Push new value into the moving average filter
    cmd_vel_maf_(new_cmd_vel);
    cmd_vel = cmd_vel_maf_.getMovingAverage();

    base_local_planner::publishPlan(global_plan_, g_plan_pub_);
  	base_local_planner::publishPlan(transformed_plan_, t_plan_pub_);


	ROS_INFO_THROTTLE_NAMED(ALP_INFO_THROTTLE, ROS_NAME, "ALP cmd_vel calculated [%.2f, %.2f | %.2f]", cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);

    // Valid cmd_vel found, return true
    return true;
}

void ArcLocalPlanner::publishPolygon(vector<rose_geometry::Point> transformed_footprint, string name)
{
	// Check if publisher with this name already exists
	if ( footprint_pubs_.find(name) == footprint_pubs_.end() ) {
 		// Not found, add it
 		ros::Publisher footprint_pub;
 		footprint_pubs_.insert(pair<string, ros::Publisher>(name, footprint_pub));
	
		if(name != "")
	 		footprint_pubs_.at(name) 		= pn_.advertise<geometry_msgs::PolygonStamped>( "footprint_stamped/" + name, 1 );      	
	 	else
	 		footprint_pubs_.at(name)		= pn_.advertise<geometry_msgs::PolygonStamped>( "footprint_stamped", 1 );      			
	} 	
	
	geometry_msgs::PolygonStamped footprint_polygon;
	footprint_polygon.header.frame_id 	= global_frame_;
 	footprint_polygon.header.stamp 		= ros::Time::now();

    for(const auto& point : transformed_footprint)
    {
    	Point32 point32;
    	point32.x = point.x;
    	point32.y = point.y;
    	footprint_polygon.polygon.points.push_back(point32);
    }

	footprint_pubs_.at(name).publish( footprint_polygon );
}

bool ArcLocalPlanner::findBestArc(int n, const vector<PoseStamped>& plan, Arc& best_arc)
{
	int index_step = 1;

	// Set search area, limit n to the end of the plan
	int closest_index = (int)getClosestWaypointIndex(global_pose_.pose, global_plan_);
	
	prev_index_ 	= min(prev_index_, closest_index);

	int half_n      = (int)ceil(n/2.0);
	int start_index = max(0, max(prev_index_ - half_n, closest_index) );
	int end_index 	= min( min( (start_index + n)*index_step, closest_index + n), (int)plan.size() );
	

	ROS_DEBUG_NAMED(ROS_NAME, "start_index: %d -> end_index: %d", start_index , end_index);

	float 	current_radius  = currentRadius();
	int 	index 			= 0;

	vector<PoseStamped> 	simulation_plan;

	vector<ArcFootprint> 	arc_footprints;
	vector<ArcFootprint> 	valid_arc_footprints;
	float min_x = 1e6;
	float min_y = 1e6;
	float max_x = -1e6;
	float max_y = -1e6;

	int dist_fails 				= 0;
	int angle_measure_fails 	= 0;
	int arrival_angle_fails 	= 0;
	int radius_diff_fails 		= 0;
	int collission_fails 		= 0;
	
	// Loop to create all arcs
	int index_range = end_index - start_index;
	float lowest_cost = 1.0/(1.0/index_range);
	for(index = start_index; index < end_index-(index_step-1); index += index_step)
	{
		if(rose_geometry::distanceXY(global_pose_.pose, plan.at(index).pose) < MIN_ARC_DIST)
		{
			dist_fails++;
			continue;				
		}


		Arc arc = findArc(global_pose_.pose, plan.at(index).pose.position);

		arc.setExtraStartDistance(0.0);
    	arc.setExtraStopDistance(0.15);

		if(arc.getAngleMeasureExtra() > MAX_ANGLE_MEASURE)
		{
			angle_measure_fails++;
			continue;
		}

		float arrival_angle_difference  = MAX_ARRIVAL_ANGLE;
		if(index + 1 < plan.size())
			arrival_angle_difference	= fabs(rose_geometry::getShortestSignedAngle(arc.getStopTangentAngle(), rose_geometry::getAngle(plan.at(index).pose.position, plan.at(index + 1).pose.position)));
		else
			arrival_angle_difference 	= 0.0;

		if( arrival_angle_difference >= MAX_ARRIVAL_ANGLE)
		{
			arrival_angle_fails++;
			continue;
		}

		ArcFootprint arc_footprint(arc, footprint_);

		// Set normalized path distance
		arc_footprint.setGlobalPathDistance(((float)index - (float)start_index + 1)/(float)index_range);
		arc_footprint.global_path_index_ = index;

		for(const auto& point : arc_footprint.getPolygonRef())
		{
			min_x = fmin(min_x, point.x);
			min_y = fmin(min_y, point.y);
			max_x = fmax(max_x, point.x);
			max_y = fmax(max_y, point.y);
		}

		arc_footprints.push_back(arc_footprint);
	}

	ROS_DEBUG_NAMED(ROS_NAME, "Found %d valid arc's out of %d. Fails: dist %d | angle_measure %d | arrival_angle %d | radius_diff %d", 	(int)arc_footprints.size(),
																																		index - start_index,
																																		dist_fails, 			
																																		angle_measure_fails, 
																																		arrival_angle_fails, 
																																		radius_diff_fails);
	// Make bounding square
	rose_geometry::Point lbs_point;
	vector<rose_geometry::Point> largest_bounding_square;
	lbs_point.x = max_x + 0.1;
	lbs_point.y = max_y + 0.1;
	largest_bounding_square.push_back(lbs_point);
	lbs_point.x = min_x - 0.1;
	lbs_point.y = max_y + 0.1;
	largest_bounding_square.push_back(lbs_point);
	lbs_point.x = min_x - 0.1;
	lbs_point.y = min_y - 0.1;
	largest_bounding_square.push_back(lbs_point);
	lbs_point.x = max_x + 0.1;
	lbs_point.y = min_y - 0.1;
	largest_bounding_square.push_back(lbs_point);

	float min_cost 						= 1e6;
	float max_cost 						= -1e6;
	float min_radius_diff				= 1e6;
	float max_radius_diff				= -1e6;
	auto lethal_cells 					= getLethalCellsInPolygon(largest_bounding_square);

	// Normalize clearances (cost)
	//! @todo OH[IMPR]: RN Give cost a correct name, in this case clearance
	for(auto& arc_footprint : arc_footprints)
	{ 
		if( not trajectoryCollidesWithCells(arc_footprint, lethal_cells) )
		{
			if(arc_footprint.getCost() < min_cost)
				min_cost = arc_footprint.getCost();

			if(arc_footprint.getCost() > max_cost)
				max_cost = arc_footprint.getCost();

			// Determine minimal and maximal arc difference
			float radius_difference = fabs(current_radius - arc_footprint.getArcRef().getSignedRadius()); 
			min_radius_diff 		= fmin(min_radius_diff, radius_difference);
			max_radius_diff 		= fmax(max_radius_diff, radius_difference);

			valid_arc_footprints.push_back(arc_footprint);
			
			// Visualize all tried arcs
			PoseStamped arc_point;
			
			for(const auto& visualize_pose : arc_footprint.getArcRef().getExtraPoses())
			{					
				arc_point.header.stamp = ros::Time::now();
				arc_point.header.frame_id = "map";
				arc_point.pose.position.x = visualize_pose.position.x;
				arc_point.pose.position.y = visualize_pose.position.y;
				simulation_plan.push_back(arc_point);
			}	
		}
		else
			collission_fails++;
	}

	ROS_DEBUG_NAMED(ROS_NAME, "Found %d valid arc(s), %d colliding footprint arc's.", (int)valid_arc_footprints.size(), collission_fails);

	ArcFootprint* best_arc_trajectory 	= NULL;
	float best_ranking 					= -1e6;
	for(const auto& arc_footprint : valid_arc_footprints)
	{
		float normalized_cost = 1.0;
		if(max_cost - min_cost != 0.0 and arc_footprint.getCost() - min_cost != 0.0)
			normalized_cost 	= (arc_footprint.getCost() - min_cost)/(max_cost - min_cost);

		// Higher ranking for arc with same radius
		float current_radius_difference = fabs(current_radius - arc_footprint.getArcRef().getSignedRadius()); 
		float normalized_arc_radius_diff = 0.0;
		if(max_radius_diff - min_radius_diff != 0.0)
			normalized_arc_radius_diff 	= fabs(current_radius_difference - min_radius_diff)/fabs(max_radius_diff - min_radius_diff);

		float ranking = ( (distance_weight_*arc_footprint.getGlobalPathDistance() ) * (clearance_weight_*normalized_cost));

		ranking *= difference_weight_*normalized_arc_radius_diff;
	
		ROS_DEBUG_NAMED(ROS_NAME, "Normalized global path distance        = %.6f", distance_weight_*arc_footprint.getGlobalPathDistance());
		ROS_DEBUG_NAMED(ROS_NAME, "Squared distance to obstacles cost     = %.6f", clearance_weight_*normalized_cost);
		ROS_DEBUG_NAMED(ROS_NAME, "normalized_arc_radius_diff %2.2f/%2.2f = %.6f", current_radius_difference, fabs(max_radius_diff - min_radius_diff), difference_weight_*normalized_arc_radius_diff);
		ROS_DEBUG_NAMED(ROS_NAME, "Resulting ranking                      = %.6f", ranking);
		
		if(ranking >= best_ranking)
		{
			ROS_DEBUG_NAMED(ROS_NAME, " New best ranking                      = %.6f", ranking);
			best_ranking 			= ranking;
			best_arc_trajectory 	= new ArcFootprint(arc_footprint);
			prev_ranking_ 			= ranking;
		}
	}
	
	// Visualize allowed arcs
	base_local_planner::publishPlan(simulation_plan, simulation_plan_pub_);

	if(best_arc_trajectory != NULL)
	{
		best_arc 				= best_arc_trajectory->getArc(); 
		prev_index_ 			= best_arc_trajectory->global_path_index_;
		publishPolygon(best_arc_trajectory->getPolygonRef(), "trajectory_poly_NOCOL");
	}
	else
	{
		prev_ranking_ 	= -1.0;
		prev_index_ 	= 0;
		ROS_DEBUG_NAMED(ROS_NAME, "findBestArc: NONE/%d chosen as best index!", n - 1);
		return false;
	}

	// Visualize stop_pose
	Pose stop_pose 	= best_arc.getStopPoseExtra();
	auto transformed_footprint_stop = footprint_;
	rotatePointsAroundOrigin(tf::getYaw(stop_pose.orientation) , transformed_footprint_stop);
	translatePoints(stop_pose.position.x , stop_pose.position.y , transformed_footprint_stop);
	publishPolygon(transformed_footprint_stop, "final_pose");

	ROS_DEBUG_NAMED(ROS_NAME, "findBestArc: %d/%d chosen as best index", prev_index_, n - 1);

	return true;
}

vector<Position2DInt> ArcLocalPlanner::getLethalCellsInPolygon(const vector<rose_geometry::Point>& polygon)
{
	// Create fake pose at map origin, in order to use getFootprintCells
	Pose map_origin_pose;
	map_origin_pose.orientation.w = 1.0;

	vector<Position2DInt> lethal_square_cells;
	vector<Position2DInt> square_cells = world_model_->getFootprintCells(map_origin_pose, toROSmsgs(polygon), true);

	ROS_DEBUG_NAMED(ROS_NAME, "%d cells found in bounding square around arc.", (int)square_cells.size());

	// Filter out only the lethal cells
	for(const auto& cell : square_cells)
	{
		if(costmap_->getCost(cell.x, cell.y) == costmap_2d::LETHAL_OBSTACLE)
			lethal_square_cells.push_back(cell);		
	}

	ROS_DEBUG_NAMED(ROS_NAME, "%d lethal cells found in bounding square around arc.", (int)lethal_square_cells.size());

	return lethal_square_cells;
}
int the_id = 1000;

vector<rose_geometry::Point> ArcLocalPlanner::getCellsPoints(	const vector<Position2DInt>& cells, 
																		bool only_center_point, 
																		const vector<rose_geometry::Point>& filter_polygon)
{
	rose_geometry::Point 			point;
	rose_geometry::Point 			corner_point;
	vector<rose_geometry::Point> 	cells_points;

	float costmap_resolution = costmap_->getResolution();
	
	for(const auto& cell : cells)
	{
		double x = (double)point.x;
		double y = (double)point.y;
		costmap_->mapToWorld(cell.x, cell.y, x, y);
		point.x = (float)x;
		point.y = (float)y;

		if(only_center_point)
		{
			corner_point.x = point.x - costmap_resolution/2.0;
			corner_point.y = point.y - costmap_resolution/2.0;
			if(filter_polygon.empty() || !isPointInPolygon(corner_point, filter_polygon))
				cells_points.push_back(corner_point);
		}
		else
		{
			corner_point.x = point.x - costmap_resolution/2.0;
			corner_point.y = point.y - costmap_resolution/2.0;
			if(filter_polygon.empty() || !isPointInPolygon(corner_point, filter_polygon))
				cells_points.push_back(corner_point);
			
			corner_point.x = point.x + costmap_resolution/2.0;
			corner_point.y = point.y + costmap_resolution/2.0;
			if(filter_polygon.empty() || !isPointInPolygon(corner_point, filter_polygon))
				cells_points.push_back(corner_point);
			
			corner_point.x = point.x + costmap_resolution/2.0;
			corner_point.y = point.y - costmap_resolution/2.0;
			if(filter_polygon.empty() || !isPointInPolygon(corner_point, filter_polygon))
				cells_points.push_back(corner_point);
			
			corner_point.x = point.x - costmap_resolution/2.0;
			corner_point.y = point.y + costmap_resolution/2.0;
			if(filter_polygon.empty() || !isPointInPolygon(corner_point, filter_polygon))
				cells_points.push_back(corner_point);
		}
	}

	publishPolygon(filter_polygon, "filter_polygon");

	
	// for(const auto& point : cells_points)
	// {
	// 	// Debug points
	// 	drawPoint(point.x, point.y, the_id++, "map", 0.0, 0.0, 1.0);
	// }
	// ROS_DEBUG_NAMED(ROS_NAME, "Found %d cells_points.", (int)cells_points.size());

	return cells_points;
}


//! @todo OH: Move this to trajectory footprint class?
bool ArcLocalPlanner::trajectoryCollidesWithCells(	TrajectoryFootprint& trajectory_footprint,
													const vector<Position2DInt>& cells)
{
	vector<rose_geometry::Point> points_to_check = getCellsPoints(cells, false);	//! @todo OH Filter footrpint??

	ROS_DEBUG_NAMED(ROS_NAME, "%d points found in bounding square around arc.", (int)points_to_check.size());

	return trajectoryCollidesWithPoints(trajectory_footprint, points_to_check);
}

//! @todo OH: Move this to trajectory footprint class?
bool ArcLocalPlanner::trajectoryCollidesWithPoints( TrajectoryFootprint& trajectory_footprint,
													const vector<rose_geometry::Point>& lethal_points)
{
	int i 				= 0;
	bool collission 	= false;
	float min_sq_dist   = 10000.0;
	float cost 		 	= 0.0;
	auto poly_points 	= trajectory_footprint.getPolygonRef();
	for(const auto& lethal_point : lethal_points)
	{
		// Inlined here because combined with min_sq_dist calculation
	    int i, j, nvert = poly_points.size();
	    bool c = false;

	    for(i = 0, j = nvert - 1; i < nvert; j = i++) 
	    {
	    	float dist = distanceSq(poly_points[i], lethal_point);
			if(dist < min_sq_dist)
				min_sq_dist = dist;

	        if( ( (poly_points[i].y > lethal_point.y ) != (poly_points[j].y > lethal_point.y) ) &&
	            (lethal_point.x < (poly_points[j].x - poly_points[i].x) * (lethal_point.y - poly_points[i].y) / (poly_points[j].y - poly_points[i].y) + poly_points[i].x)
	          )
	            c = !c;
	    }
	    if(c)
		{
			// drawPoint(lethal_point.x, lethal_point.y, i, "map", 1.0, 0.0, 0.0);
			ROS_DEBUG_NAMED(ROS_NAME, "POLY, collission found.");
			collission 	= true;
			cost 		= 0.0;
			break;
		}
	}

	cost = 1.0/(min_sq_dist);//sq_dist_sum/(/*trajectory_footprint.getPolygonRef().size() * */ poly_points.size()));
	trajectory_footprint.setCost(cost);

	trajectory_footprint.collides(false);

	if(!collission)
	{
		ROS_DEBUG_NAMED(ROS_NAME, "No collissions found, carry on soldier, average cost: %.3f", cost);
	}

	return collission;
}

/**
 * @brief Get the position tostrafe to to give us a better change at moving trough a small opening.
 * @details Select a point at the path n points ahead. Determine the orientation of the path at this point. 
 * This is done by taking the point A, n points ahead, and point B, n + 1 points ahead, and calculating the angle between these points.
 * Next the point which to strafe to will be determined by adding a vector of -length dist(current pose -> A) or max_distance.
 * The pose will 
 * 
 * @param global_pose Current position of the robot, must be in the same frame as the plan	
 * @param plan The global plan	
 * @param n How many point ahead to take the orientation of the path
 * @param range Number of points around th point n points ahead to take the average orientation from
 * @param max_distance The maximum distance the robot will be translated back from the selected point.
 * @return Target pose
 */
Pose ArcLocalPlanner::getAlignPose(		const Pose& global_pose, 
										const std::vector<PoseStamped>& plan,
										const int& n,
										const int& range,
										const float& max_distance)
{
	Pose pose = global_pose;

	// Determine the point at the path which is closest to the robot a.t.m.
	int closest_index 		= getClosestWaypointIndex(global_pose, plan);

	// Take two points. One n points ahead, one n + 1 points ahead
	//! @todo OH [IMPR]: Make this into a sperate function.
	int half_range 			= ceil((float)range / 2.0);
	int n_index 			= std::min((int)plan.size() - 1, closest_index + n);
	int target_index_low 	= std::min((int)plan.size() - 1, closest_index + n - half_range );
	int target_index_high 	= std::min((int)plan.size() - 1, closest_index + n + half_range );

	// If this is the same point, we are at the end of the path, return the pose of the final point of the path
	if(target_index_low == target_index_high)
		pose = plan.back().pose;
	else
	{
		// Select the position as the point on the path n points ahead
		pose.position = plan.at(n_index).pose.position;

		float average_orientation = 0.0;
		for(int i = target_index_low; i < target_index_high - 1; i++)
		{
			// Indicate range used for averaging
			drawPoint(plan.at(i).pose.position.x, plan.at(i).pose.position.y, i, "map", 1.0, 1.0, 0.0);
			
			// Determine direction of path
			average_orientation += rose_geometry::getAngle(plan.at(i).pose, plan.at(i + 1).pose);
		}

		// Average the oriention by dividing by the number of points
		average_orientation /= target_index_high - target_index_low;

		pose.orientation = rose_conversions::RPYToQuaterion(0.0, 0.0, average_orientation);
	}		
	
	// Translate the point a certain distance backward
	// First create a vector at the origin
	float vx = 1.0;
	float vy = 0.0;

	// Rotate the vector to align with the selected point on the path
	rose_geometry::rotateVect(&vx, &vy, tf::getYaw(pose.orientation));

	// Set the length negative to move it backward a certain amount
	rose_geometry::setVectorLengthXY(&vx, &vy, std::fmax(rose_geometry::distanceXY(global_pose.position, pose.position), max_distance));

	// Add the vector to the selected path point
	pose.position.x += vx;
	pose.position.y += vy;

	// Limit the distance such that we strafe to the selected position as far as we can from our current pose
	limitMaximalStrafeDistance(pose);

	// Display the computed align position
	drawPoint(pose.position.x, pose.position.y, (target_index_high - target_index_low + 1), "map", 0.0, 0.0, 1.0);

	return pose;
}

Pose ArcLocalPlanner::getPathDirectionPose(		const Pose& global_pose, 
												const std::vector<PoseStamped>& plan,
												const int& n)
{
	Pose pose = global_pose;

	// Determine direction of path
	int closest_index 		= getClosestWaypointIndex(global_pose, plan);
	if(closest_index < plan.size() - 1)
	{
		int upper_index = min((int)plan.size() - 1, closest_index + n + 1);
		drawPoint(plan.at(upper_index).pose.position.x, plan.at(upper_index).pose.position.y, 1, "map", 0.0, 0.0, 1.0);
		pose.orientation = rose_conversions::RPYToQuaterion(0.0, 0.0, rose_geometry::getAngle(plan.at(closest_index).pose, plan.at(upper_index).pose));
	}
	else
	{
		// At last point thus orientate to final orientation
		pose.orientation = plan.back().pose.orientation;
	}	

	return pose;
}

Pose ArcLocalPlanner::getAimAtPathPose(			const Pose& global_pose, 
												const std::vector<PoseStamped>& plan,
												const int& n)
{
	Pose pose;

	// Determine direction of path
	int closest_index = getClosestWaypointIndex(global_pose, plan);	
	
	if(closest_index  < plan.size() - 1)
	{
		int upper_index 	= min(closest_index + n, (int)plan.size() - 1);
		pose.position 		= plan.at(upper_index).pose.position;
		drawPoint(plan.at(upper_index).pose.position.x, plan.at(upper_index).pose.position.y, 1, "map", 0.0, 0.0, 1.0);
		pose.orientation 	= rose_conversions::RPYToQuaterion(0.0, 0.0, rose_geometry::getAngle(global_pose.position, plan.at(upper_index).pose.position) );
	}
	else
	{
		// At last point thus orientate to final orientation
		pose = plan.back().pose;
	}	

	return pose;
}

// 0 -> failed
// 1 -> busy
// 2 -> requested angle reached
VelCalcResult ArcLocalPlanner::calculateDriveVelocityCommand(Twist& cmd_vel)
{
	Arc best_arc;
	if( not findBestArc(100, transformed_plan_, best_arc))
	{
		// Forget the where the search of arc's was
		prev_index_ = 0;

		return FAILED; 
	}

	vector<PoseStamped> local_plan = generateLocalPlan(global_pose_.pose, best_arc);
	base_local_planner::publishPlan(local_plan, l_plan_pub_);

	float radius 	= best_arc.getSignedRadius();						
	float dist 		= distance(global_pose_.pose.position, best_arc.getStopPoint());
	float velocity 	= ((MAX_VEL_ABS - MIN_VEL_ABS)/DISTANCE_MAX_VEL)*dist + MIN_VEL_ABS;

	rose_conversions::limit(-MAX_VEL_ABS, MAX_VEL_ABS, &velocity);

	// Calculate velocity command
	Twist new_cmd_vel;
	new_cmd_vel.linear.x  = velocity;
	new_cmd_vel.linear.y  = 0.0;
	new_cmd_vel.linear.z  = 0.0;
	new_cmd_vel.angular.x = 0.0;
	new_cmd_vel.angular.y = 0.0;
	new_cmd_vel.angular.z = velocity/radius;
	

	ROS_DEBUG_NAMED(ROS_NAME, "new_cmd_vel.linear.x : %.2f", new_cmd_vel.linear.x);
	ROS_DEBUG_NAMED(ROS_NAME, "new_cmd_vel.angular.z: %.2f", new_cmd_vel.angular.z);

	// Limit angular velocity, recalculate tangential velocity
	if(new_cmd_vel.angular.z >= MIN_VEL_THETA)
	{
		new_cmd_vel.angular.z = fmin(MAX_VEL_THETA, new_cmd_vel.angular.z);
		new_cmd_vel.linear.x  = new_cmd_vel.angular.z*radius;
	}
	else if(new_cmd_vel.angular.z <= -MIN_VEL_THETA)
	{
		new_cmd_vel.angular.z = fmax(-MAX_VEL_THETA, new_cmd_vel.angular.z);
		new_cmd_vel.linear.x  = new_cmd_vel.angular.z*radius;
	}
	else
	{
		// Just drive straight if angular velocity is lower that minimal value
		new_cmd_vel.angular.z = 0.0;
	}

	ROS_DEBUG_NAMED(ROS_NAME, "LIMITED new_cmd_vel.linear.x : %.2f", new_cmd_vel.linear.x);
	ROS_DEBUG_NAMED(ROS_NAME, "LIMITED new_cmd_vel.angular.z: %.2f", new_cmd_vel.angular.z);

	//! @todo OH: Limit speeds, collission detection	
	ROS_INFO_THROTTLE_NAMED(ALP_INFO_THROTTLE, ROS_NAME, "Driving...");
	cmd_vel = new_cmd_vel;

	// Converged?
	if(isGoalReached())
		return FINISHED;
	else
		return BUSY;

}

// Returns 
// 0 -> failed
// 1 -> busy
// 2 -> requested angle reached
VelCalcResult ArcLocalPlanner::calculateRotateVelocityCommand(Twist& cmd_vel) 
{	
	float angle = rose_geometry::getShortestSignedAngle(tf::getYaw(global_pose_.pose.orientation), tf::getYaw(state_target_pose_.orientation));
	
	// Limit angle
	float max_rotation_angle = findMaxPossibleRotation(global_pose_.pose, transformed_footprint_, rose_conversions::sgn(angle));

	if(fabs(angle) > fabs(max_rotation_angle))
	{
		angle = max_rotation_angle;
	}

	// Check if initial requested, and possible, angle is above ROTATE_ANGLE_THRESHOLD (are we going to do/did we do some 'work')
	if(fabs(rose_geometry::getShortestSignedAngle(tf::getYaw(state_start_pose_.orientation), tf::getYaw(global_pose_.pose.orientation) + angle)) <= ROTATE_ANGLE_THRESHOLD)
		return FAILED;

	ROS_DEBUG_NAMED(ROS_NAME, "pose: %.4f, req_angle: %.4f, angle: %.4f, max_rotation_angle: %.4f, ROTATE_ANGLE_THRESHOLD: %.4f", 	tf::getYaw(global_pose_.pose.orientation), 
																																	tf::getYaw(state_target_pose_.orientation), 
																																	angle, 
																																	max_rotation_angle, 
																																	ROTATE_ANGLE_THRESHOLD);

	// Debug
	Pose stop_pose 	= global_pose_.pose;
	auto transformed_footprint_stop = footprint_;
	rotatePointsAroundOrigin(tf::getYaw(stop_pose.orientation) + angle , transformed_footprint_stop);
	translatePoints(stop_pose.position.x , stop_pose.position.y , transformed_footprint_stop);
	publishPolygon(transformed_footprint_stop, "final_pose");

	Twist new_cmd_vel;
	new_cmd_vel.linear.x  = 0.0;
	new_cmd_vel.linear.y  = 0.0;
	new_cmd_vel.linear.z  = 0.0;
	new_cmd_vel.angular.x = 0.0;
	new_cmd_vel.angular.y = 0.0;
	new_cmd_vel.angular.z = 1.0*angle;

	// Limit angular velocity
	if(new_cmd_vel.angular.z >= 0.0)
	{
		new_cmd_vel.angular.z = fmin(MAX_VEL_THETA_INPLACE, new_cmd_vel.angular.z);
		new_cmd_vel.angular.z = fmax(MIN_VEL_THETA_INPLACE, new_cmd_vel.angular.z);
	}
	else
	{
		new_cmd_vel.angular.z = fmax(-MAX_VEL_THETA_INPLACE, new_cmd_vel.angular.z);
		new_cmd_vel.angular.z = fmin(-MIN_VEL_THETA_INPLACE, new_cmd_vel.angular.z);
	}

	if(fabs(angle) >= ROTATE_ANGLE_THRESHOLD)
	{
		ROS_DEBUG_NAMED(ROS_NAME, "Rotating...: fabs(angle): %.2f", fabs(angle));
		cmd_vel = new_cmd_vel;
		return BUSY;
	}
	else
	{ 
		// Stop moving
		cmd_vel.linear.x  = 0.0;
		cmd_vel.linear.y  = 0.0;
		cmd_vel.linear.z  = 0.0;
		cmd_vel.angular.x = 0.0;
		cmd_vel.angular.y = 0.0;
		cmd_vel.angular.z = 0.0;

		// Check if requested angle reached
		if(fabs(rose_geometry::getShortestSignedAngle(tf::getYaw(global_pose_.pose.orientation), tf::getYaw(state_target_pose_.orientation))) < ROTATE_ANGLE_THRESHOLD)
		{
			ROS_DEBUG_NAMED(ROS_NAME, "Rotation finished.");
			return FINISHED;
		}
		else
		{
			ROS_DEBUG_NAMED(ROS_NAME, "Rotation Failed to reach requested_angle, hit maximal rotation angle.");
			// Not failed because 'work' was done
			return FINISHED;
		}
	}
}


// return = 0 is Failed 
// return = 1 is Busy 
// return = 2 is Finished 
VelCalcResult ArcLocalPlanner::calculateStrafeVelocityCommand(Twist& cmd_vel)
{
	// Do we actually want to move a certain distance?
	if(rose_geometry::distanceXY(state_start_pose_, state_target_pose_) < AT_GOAL_DIST - 0.01)	//! @todo OH: Magic number, make configurable
		return FAILED;

	// Continuosly limit the movement to what is still possible (could change due to changes in costmap)
	Pose strafe_target_pose = state_target_pose_;
	limitMaximalStrafeDistance(strafe_target_pose);

	// Visualize the point that we want to strafe to
	drawPoint(strafe_target_pose.position.x, strafe_target_pose.position.y, 1, "map", 0.0, 1.0, 0.0);
	
	float dx 	= strafe_target_pose.position.x - global_pose_.pose.position.x;
	float dy 	= strafe_target_pose.position.y - global_pose_.pose.position.y;
	float dist = rose_geometry::distanceXY(global_pose_.pose, strafe_target_pose);

	// Check if we are within strafe distance
	if(dist < AT_GOAL_DIST - 0.01)					//! @todo magic number, should be lower than AT_GOAL_DIST?
	{
		ROS_DEBUG_NAMED(ROS_NAME, "STRAFE Finished");
		cmd_vel.linear.x  = 0.0;
		cmd_vel.linear.y  = 0.0;
		cmd_vel.linear.z  = 0.0;
		cmd_vel.angular.x = 0.0; 
		cmd_vel.angular.y = 0.0;
		cmd_vel.angular.z = 0.0;
		return FINISHED;
	}

	// Calculate new velocity vector
	Twist new_cmd_vel;
	new_cmd_vel.linear.x  = 0.4*dx;
	new_cmd_vel.linear.y  = 0.4*dy;
	new_cmd_vel.linear.z  = 0.0;
	new_cmd_vel.angular.x = 0.0;
	new_cmd_vel.angular.y = 0.0;
	new_cmd_vel.angular.z = 0.0;

	// Limit the velocity vector
	if(rose_geometry::getVectorLengthXY(new_cmd_vel.linear.x, new_cmd_vel.linear.y) < MIN_VEL_ABS)
		rose_geometry::setVectorLengthXY(&new_cmd_vel.linear.x, &new_cmd_vel.linear.y, MIN_VEL_ABS);
		
	rose_geometry::limitVectorLengthXY(&new_cmd_vel.linear.x, &new_cmd_vel.linear.y, MAX_VEL_ABS);
	rose_geometry::rotateVect(&new_cmd_vel.linear.x, &new_cmd_vel.linear.y, -tf::getYaw(global_pose_.pose.orientation));

	ROS_DEBUG_NAMED(ROS_NAME, "Strafeing...");
	cmd_vel = new_cmd_vel;

	return BUSY;
}

float ArcLocalPlanner::currentRadius()
{
	float r = 0.0;
	if(local_vel_.angular.z != 0.0)
		r = sqrt(local_vel_.linear.x*local_vel_.linear.x + local_vel_.linear.y*local_vel_.linear.y)/local_vel_.angular.z;

	ROS_DEBUG_NAMED(ROS_NAME, "x, y, yaw | %.3f, %.3f, %.3f | r: %.4f | ", local_vel_.linear.x, local_vel_.linear.y, local_vel_.angular.z, r);
}

Arc ArcLocalPlanner::findArc(const Pose& start_pose, const rose_geometry::Point& check_point)
{
	return Arc(start_pose.position, tf::getYaw(start_pose.orientation), check_point);
}

// Direction positive is CCW negative is CW
float ArcLocalPlanner::findMaxPossibleRotation(	const Pose& pose, 
											const vector<rose_geometry::Point>& footprint,
											int direction)
{
	// Check direction parameter, positive is CCW negative is CW
	if(direction < -1 || direction == 0 || direction > 1)
	{
		ROS_ERROR_NAMED(ROS_NAME, "ArcLocalPlanner::findMaxPossibleRotation direction parameter should be -1 or 1.");
		return 0.0;
	}

	publishPolygon(footprint, "pose");

	ROS_DEBUG_NAMED(ROS_NAME, "tf::getYaw(pose.orientation): %.4f", tf::getYaw(pose.orientation));

	// Create polygon surrounding the circumscribed radius of the robot
	rose_geometry::Point point;
	vector<rose_geometry::Point> check_area_polygon = createBoundingPolygon(pose.position, footprint, circumscribed_radius_);

	// Get lethal cells surrounding the robot
	vector<rose_geometry::Point> lethal_points = getCellsPoints(getLethalCellsInPolygon(check_area_polygon), false, footprint);

	float radius = 0.0;

	int i;
	int j = 0;
	vector<float> angles;
	float smallest_angle 	= 2.0*M_PI;
	
	//! @todo OH: duplicate code with findMaximalStrafeDistance
	rose_geometry::Point vertex_a, vertex_b;
	for(i = 0; i < footprint.size(); i++)
	{
		vertex_a = footprint.at(i);
		if(i == footprint.size() - 1)
			vertex_b = footprint.at(0);
		else
			vertex_b = footprint.at(i + 1);

		for(const auto& lethal_point : lethal_points)
		{
			radius = distance(pose.position, lethal_point);

			if(radius <= circumscribed_radius_)
			{
				vector<rose_geometry::Point> intersections = intersectionsLineSegmentCircle(vertex_a, vertex_b, pose.position, radius);	
			
				for(const auto& intersection_point : intersections)
				{
					//drawPoint(intersection_point.x, intersection_point.y, j++, "map", 0.0, 0.0, 0.0);

					float angle_i_l = rose_geometry::getShortestSignedAngle( 	angle(pose.position, intersection_point),
																angle(pose.position, lethal_point));
														
					// Only consider angles in the correct direction.
					if(rose_conversions::sgn(angle_i_l) == rose_conversions::sgn(direction))
					{
						// angles.push_back(angle);
						if(fabs(angle_i_l) <= fabs(smallest_angle))
						{
							smallest_angle 	= angle_i_l;
						}
					}
				}		
			}		
		}
	}

	if(fabs(smallest_angle) > ROTATE_SAFETY_ANGLE)
		return smallest_angle - direction * ROTATE_SAFETY_ANGLE;
	else
		return 0.0;
}


rose_geometry::Point ArcLocalPlanner::findFootprintSteepestDescent(	const Pose& global_pose, 
																	const std::vector<Position2DInt>& footprint_cells)
{
	int dx = 0;
	int dy = 0;
	for(auto cell : footprint_cells)
	{
		auto current_cell_cost = footprint_cells_multimap_.find(make_pair(cell.x, cell.y));
		auto right_nb = footprint_cells_multimap_.find(make_pair(cell.x, cell.y - 1));
		auto below_nb = footprint_cells_multimap_.find(make_pair(cell.x - 1, cell.y));
		if(right_nb != footprint_cells_multimap_.end())
		{
			// Has right neighbourgh
			// ROS_DEBUG_NAMED(ROS_NAME, "findFootprintSteepestDescent: Self: %d RNB: %d", current_cell_cost->second, right_nb->second);
			if(right_nb->second < current_cell_cost->second)
				dy--;
			else if(right_nb->second > current_cell_cost->second)
				dy++;
		}
		if(below_nb != footprint_cells_multimap_.end())
		{
			// Has below neighbourgh
			// ROS_DEBUG_NAMED(ROS_NAME, "findFootprintSteepestDescent: Self: %d BNB: %d", current_cell_cost->second, below_nb->second);
			if(below_nb->second < current_cell_cost->second)
				dx--;
			else if(below_nb->second > current_cell_cost->second)
				dx++;
		}
	}

	rose_geometry::Point direction;
	direction.x = (float)dx;
	direction.y = (float)dy;

	ROS_DEBUG_NAMED(ROS_NAME, "findFootprintSteepestDescent: Direction: [%d, %d]", dx, dy);

	rose_geometry::setVectorLengthXY(&direction.x, &direction.y, circumscribed_radius_);

	return direction;
}

void ArcLocalPlanner::limitMaximalStrafeDistance(Pose& strafe_target)
{
	// Target selected check if distance is still safe
	rose_geometry::Point direction_vector;
	direction_vector.x = strafe_target.position.x - global_pose_.pose.position.x;
	direction_vector.y = strafe_target.position.y - global_pose_.pose.position.y;

	// Get lethal cells surrounding the robot
	vector<rose_geometry::Point>	check_area_polygon 	= createBoundingPolygon(global_pose_.pose.position, transformed_footprint_, rose_geometry::distanceXY(global_pose_.pose, strafe_target) + circumscribed_radius_);
	vector<rose_geometry::Point>	lethal_points 		= getCellsPoints(getLethalCellsInPolygon(check_area_polygon), false, transformed_footprint_);

	// Find maximal strafe distance given the direction_vector
	rose_geometry::setVectorLengthXY(	&direction_vector.x, 
										&direction_vector.y, 
										findMaximalStrafeDistance(	global_pose_.pose, 
																	transformed_footprint_, 
																	direction_vector, 
																	lethal_points, 
																	rose_geometry::distanceXY(global_pose_.pose, strafe_target)
																 )
									);

	strafe_target = global_pose_.pose;

	strafe_target.position.x += direction_vector.x;
	strafe_target.position.y += direction_vector.y;
}

float ArcLocalPlanner::findMaximalStrafeDistance(	const Pose& start_pose,
													const vector<rose_geometry::Point>& footprint,
													const rose_geometry::Point& direction,
													const vector<rose_geometry::Point>& lethal_points,
 													float distance_limit)
{
	// Check direction parameter
	if(rose_geometry::getVectorLengthXY(direction.x, direction.y) == 0.0)
	{
		ROS_ERROR_NAMED(ROS_NAME, "ArcLocalPlanner::findMaximalStrafeDistance direction vector parameter should not have length zero.");
		return 0.0;
	}

	// Set vector length such that is is larger than the distance limit
	rose_geometry::Point direction_vector = direction;
	rose_geometry::setVectorLengthXY(&direction_vector.x, &direction_vector.y, distance_limit*2.0);

	direction_vector *= -1.0;

	int i;
	int j = 0;
	float max_safe_distance = distance_limit;
	//vector<rose_geometry::Point> poly;
	rose_geometry::Point vertex_a, vertex_b;
	for(i = 0; i < footprint.size(); i++)
	{
		vertex_a = footprint.at(i);
		if(i == footprint.size() - 1)
			vertex_b = footprint.at(0);
		else
			vertex_b = footprint.at(i + 1);

		for(const auto& lethal_point : lethal_points)
		{
			vector<rose_geometry::Point> intersections = intersectionsLineSegmentLineSegment(vertex_a, vertex_b, lethal_point, (lethal_point + direction_vector));	

			for(const auto& intersection_point : intersections)
			{
				//poly.push_back(lethal_point);
				//poly.push_back(intersection_point);
				//poly.push_back(lethal_point);
				//drawPoint(intersection_point.x, intersection_point.y, j++, "map", 0.0, 0.0, 0.0);

				float dist = fmax(0.0, distance(intersection_point, lethal_point) - STRAFE_SAFETY_DIST);

				if(dist <= max_safe_distance /*|| max_safe_distance == -1.0*/)
					max_safe_distance = dist;
			}
		}
	}

	ROS_DEBUG_NAMED(ROS_NAME, "Found %d lethal_points.", (int)lethal_points.size());


	//publishPolygon(poly, "strafe_lines");

	ROS_DEBUG_NAMED(ROS_NAME, "max_safe_distance: %.2f", max_safe_distance);

	return max_safe_distance;
}

Pose ArcLocalPlanner::findStrafeTarget(		const vector<rose_geometry::Point>& footprint,
 											float distance_limit)
{
	Pose pose = global_pose_.pose;

	// Create polygon surrounding the robot with the maximal strafe distance
	rose_geometry::Point point;
	vector<rose_geometry::Point> check_area_polygon = createBoundingPolygon(global_pose_.pose.position, footprint, 2.0*distance_limit);

	// Get lethal cells surrounding the robot
	vector<rose_geometry::Point> lethal_points = getCellsPoints(getLethalCellsInPolygon(check_area_polygon), false, footprint);
	map<float, float> angle_distance_map;

	float min_dist 			= 100000.0;
	float min_dist_angle 	= 0.0;

	float start_yaw 	= tf::getYaw(global_pose_.pose.orientation);
	float yaw_step_size = (2.0*M_PI)/8.0;
	float yaw 			= start_yaw;	
	rose_geometry::Point direction_vector;
	do{
		direction_vector.x = 1.0;
		direction_vector.y = 0.0;
		rose_geometry::rotateVect(&direction_vector.x, &direction_vector.y, yaw);

		float dist = findMaximalStrafeDistance(global_pose_.pose, footprint, direction_vector, lethal_points, distance_limit);

		if(dist <= min_dist)
		{
			min_dist 		= dist;
			min_dist_angle 	= yaw;
		}

		angle_distance_map.insert(pair<float, float>(dist, yaw));

		yaw += yaw_step_size;

	}while(yaw <= start_yaw + 2.0*M_PI);


	ROS_DEBUG_NAMED(ROS_NAME, "min_dist: %.2f -> angle: %.2frad/%.2fdeg", min_dist, min_dist_angle, min_dist_angle*(180.0/(M_PI)));

	// Go in the opposite direction
	direction_vector.x = 1.0;
	direction_vector.y = 0.0;
	rose_geometry::rotateVect(&direction_vector.x, &direction_vector.y, min_dist_angle + M_PI);
	
	// Vector to furthest strafe position, limit by distance limit
	direction_vector.x = fmin(distance_limit, findMaximalStrafeDistance(global_pose_.pose, footprint, direction_vector, lethal_points, distance_limit));
	direction_vector.y = 0.0;
	rose_geometry::rotateVect(&direction_vector.x, &direction_vector.y, min_dist_angle + M_PI);

	ROS_DEBUG_NAMED(ROS_NAME,"findStrafeTarget: Target dxdy [%.2f, %.2f], distance: %.2f", direction_vector.x, direction_vector.y, min_dist);
	pose.position.x += direction_vector.x;
	pose.position.y += direction_vector.y;
	
	return pose;
}

vector<rose_geometry::Point> ArcLocalPlanner::createBoundingPolygon(	const rose_geometry::Point center,
																				const vector<rose_geometry::Point> polygon,
																				float margin)
{
	rose_geometry::Point point;
	vector<rose_geometry::Point> bounding_polygon;
	for(const auto& original_point : polygon)
	{
		point = original_point - center;
		rose_geometry::setVectorLengthXY(&point.x, &point.y, distance(center, original_point) + margin);
		point += center;
		bounding_polygon.push_back(point);
	}

	publishPolygon(bounding_polygon, "bounding_polygon");

	return bounding_polygon;
}

unsigned int ArcLocalPlanner::getClosestWaypointIndex(	const Pose& global_pose, 
														const vector<PoseStamped>& plan,
														unsigned int n)
{
	unsigned int index = 0;
	getClosestWaypoint(global_pose, plan, index, n);
	return index;
}

rose_geometry::Point ArcLocalPlanner::getClosestWaypoint(	const Pose& global_pose, 
																	const vector<PoseStamped>& plan,
																	unsigned int n)
{
	unsigned int index = 0;
	return getClosestWaypoint(global_pose, plan, index, n);
}

rose_geometry::Point ArcLocalPlanner::getClosestWaypoint(	const Pose& global_pose, 
							  										const vector<PoseStamped>& plan,
							  										unsigned int& index,
							  										unsigned int n)
{
	float min_distance 			= -1.0;
	vector<PoseStamped>::const_iterator waypoint_it;
	vector<PoseStamped>::const_iterator closest_waypoint_it 	= plan.begin();	
	for(waypoint_it = plan.begin(); waypoint_it != plan.end(); ++waypoint_it)
	{
	  	float distance = rose_geometry::distanceXY((*waypoint_it).pose, global_pose);
	  	//ROS_DEBUG_NAMED(ROS_NAME, "Pruning: index: %lu, distance: %.3f, min_distance: %.3f", waypoint_it - plan.begin(), distance, min_distance);
    	if(distance < min_distance || min_distance == -1.0)
    	{
    		min_distance 		= distance;
    		closest_waypoint_it = waypoint_it;
    	}
	}

	index = min((unsigned int)plan.size() - 1, (unsigned int)std::distance(plan.begin(), closest_waypoint_it) + n);

	return plan.at(index).pose.position;
}

vector<PoseStamped> ArcLocalPlanner::generateLocalPlan(const Pose& global_pose, Arc& arc)
{
	vector<PoseStamped> local_plan;
	rose_geometry::Point start_point 	= global_pose.position;
	rose_geometry::Point end_point 	= arc.getStopPoint();
	rose_geometry::Point center_point;

	PoseStamped indicator_point;
	indicator_point.header.stamp = ros::Time::now();
	indicator_point.header.frame_id = "map";
	indicator_point.pose.position.x = start_point.x;
	indicator_point.pose.position.y = start_point.y;
	local_plan.push_back(indicator_point);
	indicator_point.pose.position.x = end_point.x;
	indicator_point.pose.position.y = end_point.y;
	local_plan.push_back(indicator_point);

	ROS_DEBUG_NAMED(ROS_NAME, "generateLocalPlan: Radius: %.2f, center point [%.2f, %.2f]", arc.getSignedRadius(), arc.getCenter().x, arc.getCenter().y);

	for(const auto& circle_pose : arc.getExtraPoses())
	{
		indicator_point.pose.position.x = circle_pose.position.x;
		indicator_point.pose.position.y = circle_pose.position.y;
		local_plan.push_back(indicator_point);
	}
	
	ROS_DEBUG_NAMED(ROS_NAME, "generateLocalPlan: local plan created, plan has %lu points.", local_plan.size());
	
	return local_plan;
}

//! @todo OH: Move to debug/drawing helper class
void ArcLocalPlanner::drawFootprintCost(const Pose pose)
{
    drawCostMapGridCellList(world_model_->getFootprintCells(pose, toROSmsgs(footprint_), false), "footprintCost");
}

//! @todo OH: Move to debug/drawing helper class
void ArcLocalPlanner::drawCostMapGridCellList(const std::vector<Position2DInt>& cell_coordinates, std::string ros_namespace)
{
	ROS_DEBUG_NAMED(ROS_NAME, "drawCostMapGridCellList: number: %lu", cell_coordinates.size());

    visualization_msgs::MarkerArray markers;

    visualization_msgs::Marker marker;
	marker.header.frame_id = "map";
	marker.header.stamp = ros::Time();
	marker.ns = ros_namespace;
	marker.type = visualization_msgs::Marker::CUBE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.z = 0.0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = 0.025;
	marker.scale.y = 0.025;
	marker.scale.z = 0.025;
	marker.color.a = 0.25;
	marker.lifetime = ros::Duration(1.0);

	bool has_lethal_cost = false;

    for(auto point : cell_coordinates)
    {
    	marker_list_id_ = (++marker_list_id_)%1000;
    	marker.id = marker_list_id_;
    	double x, y;
    	costmap_->mapToWorld(point.x, point.y, x, y);
    	marker.pose.position.x = x;
		marker.pose.position.y = y;


		unsigned int cost = costmap_->getCost(point.x, point.y);
		//ROS_DEBUG_NAMED(ROS_NAME, "ArcLocalPlanner::drawCostMapGridCellList(%d) [%lu, %lu]->[%.2f, %.2f], cost: %d", i, point.x, point.y, x ,y, cost);

		if(cost == costmap_2d::LETHAL_OBSTACLE)
		{
			marker.color.r = 1.0;
			marker.color.g = 0.0;
			marker.color.b = 0.0;
			has_lethal_cost = true;
		}
		else if(cost == costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
		{
			marker.color.r = 0.0;
			marker.color.g = 1.0;
			marker.color.b = 0.0;
		}else if(cost == costmap_2d::NO_INFORMATION)
		{
			marker.color.r = 1.0;
			marker.color.g = 1.0;
			marker.color.b = 0.0;
		}
		else 
		{
			marker.color.r = 0.0;
			marker.color.g = 0.0;
			marker.color.b = cost/255.0;
		}
    	
    	markers.markers.push_back(marker);
    }
    
    rviz_markers_pub_.publish(markers);
}

//! @todo OH: Move to debug/drawing helper class
void ArcLocalPlanner::drawCostMapGridCell(int x, int y, int id)
{
    visualization_msgs::Marker marker;
	marker.header.frame_id = "map";
	marker.header.stamp = ros::Time();
	marker.ns = "drawCostMapGridCell";
	marker.id = id;
	marker.type = visualization_msgs::Marker::CUBE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = x;
	marker.pose.position.y = y;
	marker.pose.position.z = 0.0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = 0.05;
	marker.scale.y = 0.05;
	marker.scale.z = 0.05;
	marker.color.a = 0.25;
	marker.lifetime = ros::Duration(1.0);
	
    
	unsigned int cost = costmap_->getCost(x, y);
	// unsigned int cost = costmap_->getCost(500 + i, 500);
	if(cost == costmap_2d::LETHAL_OBSTACLE)
	{
		marker.color.r = 1.0;
		marker.color.g = 0.0;
		marker.color.b = 0.0;

	}
	else if(cost == costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
	{
		marker.color.r = 0.0;
		marker.color.g = 1.0;
		marker.color.b = 0.0;
	}else if(cost == costmap_2d::NO_INFORMATION)
	{
		marker.color.r = 1.0;
		marker.color.g = 1.0;
		marker.color.b = 0.0;
	}
	else 
	{
		marker.color.r = 0.0;
		marker.color.g = 0.0;
		marker.color.b = 1.0;
	}

    
    rviz_marker_pub_.publish(marker);
}

//! @todo OH: Move to debug/drawing helper class
void ArcLocalPlanner::drawPoint(float x, float y, int id, string frame_id, float r, float g, float b)
{
	ROS_DEBUG_NAMED(ROS_NAME, "drawPoint [%.3f, %.3f, %d]", x, y, id);

    visualization_msgs::Marker marker;
	marker.header.frame_id = frame_id;
	marker.header.stamp = ros::Time();
	marker.ns = "arc_local_planner_points";
	marker.id = id;
	marker.type = visualization_msgs::Marker::SPHERE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = x;
	marker.pose.position.y = y;
	marker.pose.position.z = 0.0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = 0.05;
	marker.scale.y = 0.05;
	marker.scale.z = 0.05;
	marker.color.r = r;
	marker.color.g = g;
	marker.color.b = b;
	marker.color.a = 0.75;
	marker.lifetime = ros::Duration(1.0);

    rviz_marker_pub_.publish(marker);
}

//! @todo OH: Move to debug/drawing helper class
void ArcLocalPlanner::drawCircle(float x, float y, float radius, int id, string frame_id, float r, float g, float b)
{
	ROS_DEBUG_NAMED(ROS_NAME, "drawCircle [%.3f, %.3f, %.3f, %d]", x, y, radius, id);

    visualization_msgs::Marker marker;
	marker.header.frame_id = frame_id;
	marker.header.stamp = ros::Time();
	marker.ns = "arc_local_planner_circles";
	marker.id = id;
	marker.type = visualization_msgs::Marker::CYLINDER;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = x;
	marker.pose.position.y = y;
	marker.pose.position.z = 0.0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = radius*2.0;
	marker.scale.y = radius*2.0;
	marker.scale.z = 0.02;
	marker.color.r = r;
	marker.color.g = g;
	marker.color.b = b;
	marker.color.a = 0.25;
	marker.lifetime = ros::Duration(20.0);

    rviz_marker_pub_.publish(marker);
}

bool ArcLocalPlanner::isGoalReached()
{
	ROS_DEBUG_NAMED(ROS_NAME, "ArcLocalPlanner::isGoalReached");

	if (!isInitialized()) {
        ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
        return false;
    }

    if(transformed_plan_.empty())
		return false;

   	float distance_squared  = rose_geometry::distanceXYsquared(transformed_plan_.back(), global_pose_);
    float angle_difference  = fabs(rose_geometry::getShortestSignedAngle(tf::getYaw(global_pose_.pose.orientation), tf::getYaw(transformed_plan_.back().pose.orientation)));
    if(distance_squared < AT_GOAL_DIST*AT_GOAL_DIST*2.0) //! @todo magic number
    {
    	auto goal_point = global_plan_.back();
    	global_plan_.clear();
    	global_plan_.push_back(goal_point);
    }

    if(distance_squared < AT_GOAL_DIST*AT_GOAL_DIST && angle_difference < AT_GOAL_ANGLE) 
    {

    	ROS_INFO_THROTTLE_NAMED(ALP_INFO_THROTTLE, ROS_NAME, "Close enough to goal, going on until orientaion or distance error starts to increase, distance: %.6fm/distance_squared_prev_: %.6fm, orientation difference: %.6fdeg/angle_difference_prev_: %.6fdeg", sqrt(distance_squared), sqrt(distance_squared_prev_), angle_difference*(180/M_PI), (angle_difference_prev_)*(180.0/M_PI));

    	if(distance_squared > distance_squared_prev_  || angle_difference > angle_difference_prev_ ) //! @todo magic numbers
    	{
	    	ROS_INFO_NAMED(ROS_NAME, "Goal reached, distance: %.6fm, orientation difference: %.6fdeg", sqrt(distance_squared),  angle_difference*(180/M_PI));
	    	state_ = DRIVE;
		  	prev_state_ = DRIVE;
	    	transformed_plan_.clear();
	    	return true;
	    }
    }

	ROS_INFO_THROTTLE_NAMED(ALP_INFO_THROTTLE, ROS_NAME, "Goal not yet reached, distance: %.6fm, orientation difference: %.6fdeg", sqrt(distance_squared),  angle_difference*(180/M_PI));

	distance_squared_prev_ = distance_squared;
	angle_difference_prev_ = angle_difference;

	return false;
}


bool ArcLocalPlanner::isInitialized()
{
	return initialized_;
}

};
