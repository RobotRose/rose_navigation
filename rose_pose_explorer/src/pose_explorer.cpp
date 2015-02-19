/***********************************************************************************
* Copyright: Rose B.V. (2014)
*
* Revision History:
*	Author: Mathijs de Langen
*	Date  : 2014/07/21
* 		- File created.
*
* Description:
*	This node calculates all possible poses defined by two point clouds.
*	
*	The input point cloud *allowed points* defines allowed points in a certain 
*	frame. Input *target points* defines where these allowed points should be. The
*	allowed points are mapped onto the target points and from this, possbile poses are 
*	calculated.
*
* Service
* Input: Two point clouds
* Output: A vector of possible poses.
* 
***********************************************************************************/
#include "rose_pose_explorer/pose_explorer.hpp"

PoseExplorer::PoseExplorer( std::string name, ros::NodeHandle n )
	: name_ (name)
	, n_ (n)
{
	ROS_INFO_NAMED(ROS_NAME, "Starting PoseExplorer...");

	reachability_service_ = n_.advertiseService( name_ + "/calculate_reachability", &PoseExplorer::CB_calculateReachability, this );

	createMarkerPublishers();

	ROS_INFO_NAMED(ROS_NAME, "PoseExplorer started");
}

PoseExplorer::~PoseExplorer()
{
	
}

bool PoseExplorer::CB_calculateReachability ( rose_pose_explorer::reachable_poses::Request  &req,	
											  rose_pose_explorer::reachable_poses::Response &res )
{
	ROS_DEBUG_NAMED(ROS_NAME, "PoseExplorer::CB_calculateReachability");
	transformToBaselink( req.target_points );
	transformToBaselink( req.allowed_points );

	for ( auto& target_point : req.target_points.points )
	{
		vector<Point32> projected_points;
		for ( auto& allowed_point : req.allowed_points.points )
			projected_points.push_back(projectPointOnPoint(allowed_point, target_point));

		vector<PoseStamped> rotated_points;
		vector<PoseStamped> rotated_points_total;
		for ( auto& point : projected_points )
		{
			rotated_points = rotatePointAroundTarget( point, target_point );
			rotated_points_total.insert( rotated_points_total.end(), rotated_points.begin(), rotated_points.end());			
		}

		//! @todo MdL: 'Overlay' found circles for different target points
		res.reachable_points = combinePoints(res.reachable_points, rotated_points_total);
	}

	publishMarkers( req.target_points, req.allowed_points, res.reachable_points);

	return true;
}

vector<PoseStamped> PoseExplorer::combinePoints( const vector<PoseStamped> points1, const vector<PoseStamped> points2 )
{
	vector<PoseStamped> points_combined;
	points_combined.insert(points_combined.end(), points1.begin(), points1.end() );
	points_combined.insert(points_combined.end(), points2.begin(), points2.end() );
	return points_combined;
}

vector<PoseStamped> PoseExplorer::rotatePointAroundTarget( const Point32 point, const Point32 target, const int nr_points_on_circle )
{
	vector<PoseStamped> poses;
	for ( double angle = 0 ; angle < 2*M_PI ; angle += (2*M_PI)/nr_points_on_circle )
		poses.push_back(rotate( target, point, angle ));

	return poses;
}

PoseStamped PoseExplorer::rotate( const Point32 vector_start, const Point32 vector_end, const double angle )
{
	// translate to origin
 	double vector_x = vector_end.x - vector_start.x;
	double vector_y = vector_end.y - vector_start.y;

	// rotate
	rose_geometry::rotateVect(&vector_x, &vector_y, angle);

	// translate back to point
	PoseStamped result;

	result.header.stamp     = ros::Time::now();
	result.header.frame_id  = "base_link";

	result.pose.position.x  = vector_start.x + vector_x;
	result.pose.position.y  = vector_start.y + vector_y;
	result.pose.position.z  = vector_end.z;

	// Rotate orientation 
	result.pose.orientation = tf::createQuaternionMsgFromYaw(angle);

	return result;	
}

Point32 PoseExplorer::projectPointOnPoint( const Point32 allowed, const Point32 target )
{
	Point32 result;
	result.x 	= target.x - allowed.x;
	result.y 	= target.y - allowed.y;
	result.z 	= target.z - allowed.z;
	//! @MdL: Add orientation?
	//! 
	return result;
}

void PoseExplorer::transformToBaselink( PointCloud& point_cloud )
{
	PointCloud result;
	try
	{
		tf_.transformPointCloud("base_link", point_cloud, result);
	}
	catch( tf::TransformException& ex )
	{
		ROS_ERROR("Received an exception: %s", ex.what());
	}

	point_cloud = result;
}

void PoseExplorer::createMarkerPublishers()
{
	required_poses_pub_ = n_.advertise<MarkerArray>( name_  + "/required_positions", 1);
	allowed_points_pub_ = n_.advertise<MarkerArray>( name_  + "/allowed_positions", 1);
	target_points_pub_  = n_.advertise<MarkerArray>( name_  + "/target_positions", 1);
}

void  PoseExplorer::publishMarkers( const PointCloud target_points,  const PointCloud allowed_points,  const vector<PoseStamped> required_poses )
{
	publishTargetPoints(target_points);
	publishAllowedPoints(allowed_points);
	publishReachableMarkers(required_poses);
}

MarkerArray  PoseExplorer::getMarkers( const PointCloud point_cloud )
{
	MarkerArray markers;
   	int i = 0;

    for ( auto& point : point_cloud.points )
    {
		visualization_msgs::Marker marker;
	    marker.header 	 = point_cloud.header;
	    marker.ns        = "markers" + point_cloud.header.seq;

	    marker.pose.position.x    = point.x;
	    marker.pose.position.y    = point.y;
	    marker.pose.position.z    = point.z;
	    marker.pose.orientation.w = 1.0f;
	    
	    marker.action    = visualization_msgs::Marker::ADD;
	    marker.type      = visualization_msgs::Marker::SPHERE;
	    marker.id        = i++;
	    marker.scale.x   = 0.03;
	    marker.scale.y   = 0.03;
	    marker.scale.z   = 0.03;
	    marker.color.g   = 1.0f;
	    marker.color.a   = 1.0f;

	    marker.lifetime  = ros::Duration(10);

		markers.markers.push_back(marker);
    }

    ROS_DEBUG_NAMED(ROS_NAME, "Publishing %d markers", (int)markers.markers.size() );

    return markers;
}

void PoseExplorer::publishTargetPoints( const PointCloud target_points)
{
	MarkerArray markers = getMarkers(target_points);
	target_points_pub_.publish(markers);
}

void PoseExplorer::publishAllowedPoints( const PointCloud allowed_points)
{
	MarkerArray markers = getMarkers(allowed_points);
	allowed_points_pub_.publish(markers);
}

void PoseExplorer::publishReachableMarkers( const vector<PoseStamped> required_poses )
{
	MarkerArray arrows;
   	int i = 0;

    for ( auto& pose_stamped : required_poses )
    {
		visualization_msgs::Marker arrow;
	    arrow.header 	= pose_stamped.header;
	    arrow.ns        = "reachable_markers";
	    arrow.pose		= pose_stamped.pose;
	    
	    arrow.action    = visualization_msgs::Marker::ADD;
	    arrow.type      = visualization_msgs::Marker::ARROW;
	    arrow.id        = i++;
	    arrow.scale.x   = 0.05;
	    arrow.scale.y   = 0.01;
	    arrow.scale.z   = 0.01;
	    arrow.color.g   = 1.0f;
	    arrow.color.a   = 1.0f;

	    arrow.lifetime  = ros::Duration(3);

		arrows.markers.push_back(arrow);
    }
    
    ROS_DEBUG_NAMED(ROS_NAME, "Publishing %d arrows", (int)arrows.markers.size() );
	required_poses_pub_.publish(arrows);
}