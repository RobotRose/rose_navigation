/***********************************************************************************
* Copyright: Rose B.V. (2014)
*
* Revision History:
*	Author: Okke Hendriks
*	Date  : 2014/07/09
* 		- File created.
*
* Description:
*	Implementation of the ArcFootprint class.
* 
***********************************************************************************/

#include "rose_arc_local_planner/arc_footprint.hpp"

ArcFootprint::ArcFootprint(const Arc& arc, const vector<rose_geometry::Point>& footprint)
	: TrajectoryFootprint()
{
	initialize(arc, footprint, 0.1);			// Default to 0.1 [m]
}

ArcFootprint::ArcFootprint(const Arc& arc, const vector<rose_geometry::Point>& footprint, float resolution)
	: TrajectoryFootprint()
{
	initialize(arc, footprint, resolution);
}

bool ArcFootprint::initialize(const Arc& arc, const vector<rose_geometry::Point>& footprint, float resolution)
{
	footprint_ 	= footprint;
	resolution_ = resolution;
	arc_ 		= arc;
	global_path_index_ = -1;
	return generatePolygon();
}

ArcFootprint::~ArcFootprint()
{}


Arc ArcFootprint::getArc() const
{
	return arc_;
}

const Arc& ArcFootprint::getArcRef() const
{
	return arc_;
}

rose_geometry::Point ArcFootprint::getEndPoint() const
{
	return arc_.getStopPoint();
}

// Assumes footprint is a square CCW polygon starting in the +x +y corner of the footprint
bool ArcFootprint::generatePolygon()
{
	if(footprint_.size() != 4)
		return false;

	if(arc_.getAngleMeasure() >= M_PI)
	{
		ROS_ERROR_NAMED(ROS_NAME, "This function, 'ArcLocalPlanner::arcCollidesWithPoints', currently only works for arc's with an angle measure of less than 180 degrees."); 	//! @todo OH: Make it work always!
		return true;
	}

	Arc outer_arc 	= arc_;
	Arc middle_arc 	= arc_;
	Arc inner_arc 	= arc_;

	// Get start and stop pose
	geometry_msgs::Pose start_pose 	= middle_arc.getStartPose();
	geometry_msgs::Pose stop_pose 	= middle_arc.getStopPoseExtra();
	// Get footprint at start and end of middle_arc
	// Make copy of footprint
	auto transformed_footprint_start 	= footprint_;
	auto transformed_footprint_stop		= footprint_;
	// Rotate footprint around origin
	rose_geometry::rotatePointsAroundOrigin(tf::getYaw(start_pose.orientation), transformed_footprint_start);
	rose_geometry::rotatePointsAroundOrigin(tf::getYaw(stop_pose.orientation) , transformed_footprint_stop);
	// Translate footprint
	rose_geometry::translatePoints(start_pose.position.x, start_pose.position.y, transformed_footprint_start);
	rose_geometry::translatePoints(stop_pose.position.x , stop_pose.position.y , transformed_footprint_stop);
	
	float outer_radius;
	float inner_radius;

	float platform_half_width  	= fabs(footprint_[0].y - footprint_[2].y)/2.0; 				// 0.62
	float platform_half_length 	= fabs(footprint_[0].x - footprint_[2].x)/2.0;				// 0.82
	float inscribed_radius 		= fmin(platform_half_width, platform_half_length);

	// Adapt radi to corner radi, because of square base
	float outer_corner_radius  		= rose_conversions::sgn(middle_arc.getSignedRadius()) * sqrt(pow(platform_half_width + middle_arc.getAbsRadius(), 2.0) + pow(platform_half_length, 2.0));
	float outer_corner_angle 		= atan(platform_half_length/outer_corner_radius);
	outer_radius 					= outer_corner_radius;

	if(middle_arc.getAbsRadius() >= inscribed_radius)	
	{
		if(middle_arc.getSignedRadius() >= 0)	
			inner_radius = fmax(0.0, middle_arc.getSignedRadius() - inscribed_radius);
		else
			inner_radius = fmin(0.0, middle_arc.getSignedRadius() + inscribed_radius);
	}
	else
	{
		inner_radius = 0.0;
		rose_geometry::Point inner_point;
		inner_point.x = inscribed_radius;
		inner_point.y = 0.0;
		if(middle_arc.isCW())
			rose_geometry::rotateVect(&inner_point.x, &inner_point.y, tf::getYaw(start_pose.orientation) - 0.5*M_PI);
		else
			rose_geometry::rotateVect(&inner_point.x, &inner_point.y, tf::getYaw(start_pose.orientation) +0.5*M_PI);

		inner_point.x += middle_arc.getStartPose().position.x;
		inner_point.y += middle_arc.getStartPose().position.y;
		inner_arc.setCenter(inner_point);
	}

	outer_arc.setSignedRadius(outer_radius);
	inner_arc.setSignedRadius(inner_radius);



	// Create footprint trajectory polygon
	if(middle_arc.isCW())
	{
		// Rear right corners of base
		polygon_.push_back(transformed_footprint_start[1]);
		// Rear left corners of base
		polygon_.push_back(transformed_footprint_start[2]);
	}
	else
	{
		// Rear left corners of base
		polygon_.push_back(transformed_footprint_start[2]);
		// Rear right corners of base
		polygon_.push_back(transformed_footprint_start[1]);
	}

	// Inner arc
	BOOST_FOREACH(const auto& arc_point, inner_arc.getArcPoints(resolution_))
		polygon_.push_back(arc_point);

	if(middle_arc.isCW())
	{
		// Front right corners of base
		polygon_.push_back(transformed_footprint_stop[3]);
		// Front left corners of base
		polygon_.push_back(transformed_footprint_stop[0]);
	}
	else
	{
		// Front left corners of base
		polygon_.push_back(transformed_footprint_stop[0]);
		// Front right corners of base
		polygon_.push_back(transformed_footprint_stop[3]);
	}

	// Outer arc
	BOOST_REVERSE_FOREACH(const auto& arc_point, outer_arc.getArcPoints(outer_arc.getStartAngle() - outer_corner_angle, outer_arc.getStopAngleExtra() + outer_corner_angle, resolution_))
		polygon_.push_back(arc_point);

	return true;
}
