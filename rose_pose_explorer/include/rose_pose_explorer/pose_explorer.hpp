/***********************************************************************************
* Copyright: Rose B.V. (2014)
*
* Revision History:
*  Author: Mathijs de Langen
*  Date  : 2014/07/21
*     - File created.
*
* Description:
*   This node calculates all possible poses defined by two point clouds.
*   
*   The input point cloud *allowed points* defines allowed points in a certain 
*   frame. Input *target points* defines where these allowed points should be. The
*   allowed points are mapped onto the target points and from this, possbile poses are 
*   calculated.
*
* Service
* Input: Two point clouds
* Output: A vector of possible poses.
* 
***********************************************************************************/
#ifndef POSE_EXPLORER_HPP
#define POSE_EXPLORER_HPP

#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h> 
#include <geometry_msgs/PointStamped.h> 
#include <geometry_msgs/Pose.h> 

#include <tf/transform_listener.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "rose_pose_explorer/reachable_poses.h"
#include "rose_common/common.hpp"
#include "rose_geometry/geometry.hpp"

using geometry_msgs::Point32;
using geometry_msgs::PointStamped;
using geometry_msgs::PoseStamped;
using sensor_msgs::PointCloud;
using std::vector;
using visualization_msgs::MarkerArray;

class PoseExplorer
{

  public:
    PoseExplorer( std::string name, ros::NodeHandle n );
    ~PoseExplorer();

  private:
    vector<PoseStamped> combinePoints( const vector<PoseStamped> points1, const vector<PoseStamped> points2 );
    vector<PoseStamped> rotatePointAroundTarget( const Point32 pose, const Point32 target, const int nr_points_on_circle = 32 );
    PoseStamped         rotate( const Point32 vector_start, const Point32 vector_end, const double angle );

    Point32             projectPointOnPoint( const Point32 allowed, const Point32 target );

    void transformToBaselink( PointCloud& point_cloud );
    bool CB_calculateReachability ( rose_pose_explorer::reachable_poses::Request  &req, 
                                    rose_pose_explorer::reachable_poses::Response &res );

    // Visulization
    void        createMarkerPublishers();
    void        publishMarkers( const PointCloud target_points,  
                                const PointCloud allowed_points,  
                                const vector<PoseStamped> required_poses );
    void        publishReachableMarkers( const vector<PoseStamped> required_poses );
    void        publishTargetPoints( const PointCloud target_points);
    void        publishAllowedPoints( const PointCloud allowed_points);
    MarkerArray getMarkers( const PointCloud point_cloud );

  	std::string 	        name_;
  	ros::NodeHandle         n_;

    ros::Publisher          allowed_points_pub_;
    ros::Publisher          required_poses_pub_;
    ros::Publisher          target_points_pub_;
    ros::ServiceServer      reachability_service_;
    tf::TransformListener   tf_;
};

#endif //POSE_EXPLORER_HPP

