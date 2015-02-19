/***********************************************************************************
* Copyright: Rose B.V. (2014)
*
* Revision History:
*	Author: Okke Hendriks
*	Date  : 2014/07/09
* 		- File created.
*
* Description:
*	description
* 
***********************************************************************************/

#include "arc_local_planner/trajectory_footprint.hpp"

using std::vector;

TrajectoryFootprint::TrajectoryFootprint()
	: cost_(0.0)
	, global_path_distance_(0.0)
	, collides_(false)
{}

TrajectoryFootprint::~TrajectoryFootprint()
{}


void TrajectoryFootprint::setCost(float cost)
{
	cost_ = cost;
}

float TrajectoryFootprint::getCost() const
{
	return cost_;
}

void TrajectoryFootprint::setGlobalPathDistance(float distance)
{
	global_path_distance_ = distance;
}

float TrajectoryFootprint::getGlobalPathDistance() const
{
	return global_path_distance_;
}

void TrajectoryFootprint::setPolygon(const vector<rose_geometry::Point>& polygon) 
{
	polygon_ = polygon;
}

vector<rose_geometry::Point> TrajectoryFootprint::getPolygon() const
{
	return polygon_;
}

const vector<rose_geometry::Point>& TrajectoryFootprint::getPolygonRef() const
{
	return polygon_;
}

bool TrajectoryFootprint::collides() const
{
	return collides_;
}

void TrajectoryFootprint::collides(bool collides)
{
	collides_ = collides;
}

rose_geometry::Point TrajectoryFootprint::getEndPoint() const
{
	ROS_WARN_NAMED(ROS_NAME, "Function getEndPoint() not implemented in base class.");
	rose_geometry::Point point;
	return point;
}
