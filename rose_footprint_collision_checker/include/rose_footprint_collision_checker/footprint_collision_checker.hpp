/***********************************************************************************
* Copyright: Rose B.V. (2014)
*
* Revision History:
*   Author: Okke Hendriks
*   Date  : 2014/09/10
*       - File created.
*
* Description:
*   Given a number of points, a velocity twist message and a footprint. 
*   Calculate the distance to collision.
* 
***********************************************************************************/

#ifndef FOOTPRINT_COLLISION_CHECKER_HPP
#define FOOTPRINT_COLLISION_CHECKER_HPP

#include <mutex>
#include <boost/progress.hpp>

#include <ros/ros.h>

#include <tf/tf.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PolygonStamped.h>

#include <tf/transform_listener.h> 

#include <visualization_msgs/Marker.h>

#include "rose_common/common.hpp"
#include "ros_name/ros_name.hpp"
#include "rose_geometry/point.hpp"
#include "rose_geometry/stamped.hpp"
#include "rose_transformations/transformations.hpp"
#include "rose_geometry/geometry.hpp"
#include "rose_conversions/conversions.hpp"

#include "polyclipping/clipper.hpp"

using namespace ClipperLib;

#define POLYGON_PRECISION 1000000.0

typedef rose_geometry::Point                    Vertex;
typedef rose_geometry::Stamped<Vertex>          StampedVertex;
typedef std::vector<Vertex>                     Vertices;
typedef Vertices                                Polygon;
typedef std::vector<Polygon>                    Polygons;
typedef std::vector<StampedVertex>              StampedVertices;
typedef StampedVertices                         StampedPolygon;
typedef std::vector<StampedPolygon>             StampedPolygons;
typedef std::vector<geometry_msgs::PoseStamped> Trajectory;

class FootprintCollisionChecker
{
  public:
    FootprintCollisionChecker();
    ~FootprintCollisionChecker();

    void            loadParameters();

    bool            setFrameOfMotion(const geometry_msgs::PoseStamped& frame_of_motion);
    bool            setFootprint(const std::vector<rose_geometry::Point>& new_footprint);
    std::string     getFrameOfMotion();
    std::vector<rose_geometry::Point>  getFootprint();
    bool            addPoints(const StampedVertices& new_lethal_points);
    bool            clearPoints();
    StampedVertices transformPointsToFrame(const StampedVertices& stamped_points, const std::string& frame_id);
    bool            checkVelocity(const geometry_msgs::Twist& vel, const float& forward_t);
    bool            checkTrajectory(const Trajectory& trajectory);
    bool            pathCollission(const Path& path, const StampedVertices& lethal_points);
    bool            polygonCollission(const Polygon& polygon, const StampedVertices& lethal_points);
    Trajectory      calculatePoseTrajectory(const geometry_msgs::Twist& vel, const float& dt, const float& forward_t, const float& max_distance);
    Polygon         getPolygonAtPose( const geometry_msgs::PoseStamped& stamped_pose, 
                                const std::vector<rose_geometry::Point>& footprint);
    bool            setMaxDistance(float max_distance);
    bool            setMaxForwardSimTime(float max_forward_sim_time);

    void            showCollisions();
    void            hideCollisions();

  protected:
    StampedVertices             lethal_points_;
    StampedVertices             transformed_lethal_points_;
    Polygon                     footprint_;
    geometry_msgs::PoseStamped  frame_of_motion_;
    float                       max_distance_;
    float                       max_forward_sim_time_;

  private:
    typedef std::vector<rose_geometry::Point> polygon;
    typedef std::vector<polygon> polygons;

    Path            getSweptPolygonPath(const Trajectory& frame_of_motion_trajectory, const Polygon& polygon);
    Polygon         getSweptPolygonPolygon(const Trajectory& frame_of_motion_trajectory, const Polygon& polygon);
    Paths           getSweptPolygonSubPaths(const Trajectory& frame_of_motion_trajectory, const Polygon& polygon);
    Polygons        getSweptPolygonSubPolys(const Trajectory& frame_of_motion_trajectory, const Polygon& polygon);
    Path            unionPaths(const Paths& paths);
    Polygon         unionPolygons(const Polygons& polygons);
    Path            trajectoryToPath(const Trajectory& trajectory);
    Path            polygonToPath(const Polygon& polygon);
    Paths           polygonsToPaths(const Polygons& polygons);
    Polygon         pathToPolygon(const Path& path);
    Polygons        pathsToPolygons(const Paths& paths);

    void getTrajectoryDistance(const Trajectory& trajectory, float& euclidean_distance, float& rotation);
    void getPoseDistance(const geometry_msgs::PoseStamped& pose_a, const geometry_msgs::PoseStamped& pose_b, float& euclidean_distance, float& rotation);
    Polygon createAABB(const Polygon& polygon, float margin);
    Polygon createAABB(const Path& path, float margin);
    bool inAABB(const Vertex& point, const Polygon& aabb);
    
    void drawPose(ros::NodeHandle& n, const geometry_msgs::PoseStamped& stamped_pose, int id, float r, float g, float b);
    void publishPolygon(polygon transformed_footprint, std::string frame, std::string name);
    void drawPoint(const StampedVertex& stamped_point, int id, float r, float g, float b);

    ros::NodeHandle             n_;
    tf::TransformListener       tf_listener_;
    ros::Publisher              rviz_marker_pub_;
    std::map<std::string, ros::Publisher> footprint_pubs_;

    std::mutex                  points_mutex_;

    bool    show_collissions_;

    boost::timer* timer;
};

#endif // FOOTPRINT_COLLISION_CHECKER_HPP 
