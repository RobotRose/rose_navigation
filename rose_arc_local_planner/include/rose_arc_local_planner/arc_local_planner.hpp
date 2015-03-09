/***********************************************************************************
* Copyright: Rose B.V. (2014)
*
* Revision History:
*   Author: Okke Hendriks
*   Date  : 2014/02/26
*       - File created.
*
* Description:
*   Rose B.V. local planner that connects the poses in a given local path using arcs
*   
* 
***********************************************************************************/
#ifndef ARC_LOCAL_PLANNER_H
#define ARC_LOCAL_PLANNER_H

#include <ros/ros.h>

#include <vector>
#include <string>
#include <cmath>

//for obstacle data access
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/cost_values.h>

#include <base_local_planner/footprint_helper.h>
#include <base_local_planner/local_planner_util.h>
#include <base_local_planner/odometry_helper_ros.h>

#include <base_local_planner/costmap_model.h>
#include <base_local_planner/trajectory.h>
#include <base_local_planner/Position2DInt.h>
#include <base_local_planner/BaseLocalPlannerConfig.h>
#include <base_local_planner/trajectory_planner_ros.h>
#include <base_local_planner/goal_functions.h>
#include <base_local_planner/map_cell.h>
#include <base_local_planner/map_grid.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PolygonStamped.h>


#include <boost/foreach.hpp>


#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <pluginlib/class_list_macros.h>
#include <nav_msgs/Path.h>

#include <nav_core/base_local_planner.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <costmap_2d/cost_values.h>

#include "rose_arc_local_planner/arc_local_planner_costmap_model.hpp"
#include "rose_arc_local_planner/arc.hpp"
#include "rose_arc_local_planner/arc_footprint.hpp"
#include "rose_arc_local_planner/trajectory_footprint.hpp"

#include "rose_common/common.hpp"
#include "rose_conversions/conversions.hpp"
#include "rose_geometry/point.hpp"
#include "rose_twist_moving_average_filter/twist_maf.hpp"

#include "server_multiple_client/server_multiple_client.hpp"
#include "rose_footprint_collision_checker/footprint_collision_checker.hpp"

using std::string;
using std::vector;

using geometry_msgs::Point32;
using geometry_msgs::Pose;
using geometry_msgs::PoseStamped;
using geometry_msgs::Twist;
using base_local_planner::Position2DInt;

#define ALP_INFO_THROTTLE       0.001   // Controls the rate at which the local planner info messages are printed

namespace rose_navigation{

enum StatemachineState
{
    DRIVE, AIM_AT_PATH, ALIGN, STRAFE_TO_PATH, STRAFE_TO_OPEN_AREA
};

enum VelCalcResult
{
    FAILED, BUSY, FINISHED
};

struct TrajectoryScore
{
    Twist       velocity;
    Trajectory  trajectory;
    float       score;
    float       cost;
};


class Timing
{
public:
    Timing(const std::string name, const int& average_over)
        : name_(name)
        , average_over_(average_over)
    {}

    ~Timing()
    {}

    void show(const float& rate = 0.0)
    {
        purge();
        
        if( start_times.size() != finish_times.size())
        {
            ROS_ERROR_NAMED(ROS_NAME, "Timing: The number of start times is not equal to the number of stop times.");
        }

        float sum = 0;
        int samples = start_times.size();
        for(int i = 0; i < samples; i++)
        {
            sum += finish_times.at(i).toSec() - start_times.at(i).toSec();
        }

        sum /= (float)samples;


        if(rate != 0.0)
            ROS_INFO_THROTTLE_NAMED(rate, ROS_NAME, "Timing '%s', timing over %d samples, average: %.6f", name_.c_str(), samples, sum);
        else
            ROS_INFO_NAMED(ROS_NAME, "Timing '%s', timing over %d samples, average: %.6f", name_.c_str(), samples, sum);
    }

    void start()
    {
        start_times.push_back(ros::Time::now());
    }

    void stop()
    {
        finish_times.push_back(ros::Time::now());
    }

    void reset()
    {
        start_times.clear();
        finish_times.clear();
    }

private:
    std::string name_;
    int average_over_;
    std::vector<ros::Time> start_times;
    std::vector<ros::Time> finish_times;

    void purge()
    {
        if(start_times.size() > average_over_)
            start_times.erase(start_times.begin(), std::next(start_times.begin(), start_times.size() - average_over_ - 1 ));

        if(finish_times.size() > average_over_)
            finish_times.erase(finish_times.begin(), std::next(finish_times.begin(), finish_times.size() - average_over_ - 1 ));
    }
};

class ArcLocalPlanner : public nav_core::BaseLocalPlanner
{
  public:
    ArcLocalPlanner();
    ArcLocalPlanner(string name, tf::TransformListener* tf_listener, costmap_2d::Costmap2DROS* costmap_ros);
    ~ArcLocalPlanner();

    void initialize(string name, tf::TransformListener* tf_listener, costmap_2d::Costmap2DROS* costmap_ros);
    bool setPlan(const vector<PoseStamped>& plan);
    bool computeVelocityCommands(Twist& cmd_vel);
    bool isGoalReached();
   
   private:
    void    loadParameters();
    void    CB_alarm_changed(const bool& new_alarm_value);
    bool    isInitialized();
    void    publishPolygon(vector<rose_geometry::Point> transformed_footprint, string name);

    unsigned int    getClosestWaypointIndex(    const PoseStamped& global_pose, 
                                                const vector<PoseStamped>& plan,
                                                unsigned int n = 0);

    rose_geometry::Point getClosestWaypoint(    const PoseStamped& global_pose, 
                                                const vector<PoseStamped>& plan,
                                                unsigned int n = 0);

    rose_geometry::Point getClosestWaypoint(    const PoseStamped& global_pose, 
                                                const vector<PoseStamped>& plan,
                                                unsigned int& index, 
                                                unsigned int n = 0); 

    vector<PoseStamped>  generateLocalPlan(const Pose& global_pose, Arc& arc);
    
    void    drawFootprintCost(Pose global_pose);
    void    drawCostMapGridCellList(const std::vector<Position2DInt>& points, std::string ros_namespace);
    void    drawCostMapGridCell(int x, int y, int id);
    void    drawPoint(float x, float y, int id, string frame_id, float r, float g, float b);
    void    drawCircle(float x, float y, float radius, int id, string frame_id, float r, float g, float b);

    bool    findBestCommandVelocity(const vector<PoseStamped>& plan, Twist& best_cmd_vel);
    rose_geometry::Point   findFootprintSteepestDescent(  const Pose& global_pose, 
                                                                    const std::vector<Position2DInt>& footprint_cells);
    Pose   findStrafeTarget(    const vector<rose_geometry::Point>& footprint,
                                float distance_limit);
    
    float   findMaxPossibleRotation(const Pose& pose, const vector<rose_geometry::Point>& footprint, int direction);
    void    limitMaximalStrafeDistance(Pose& pose);
    float   findMaximalStrafeDistance(  const Pose& pose,
                                        const vector<rose_geometry::Point>& footprint,
                                        const rose_geometry::Point& direction,
                                        const vector<rose_geometry::Point>& lethal_points,
                                        float distance_limit);

    vector<rose_geometry::Point> createBoundingPolygon(   const rose_geometry::Point center,
                                                                    const vector<rose_geometry::Point> polygon,
                                                                    float margin);
    Pose    getAlignPose(           const PoseStamped& global_pose, 
                                    const std::vector<PoseStamped>& plan,
                                    const int& n,
                                    const int& range,
                                    const float& max_distance);

    Pose    getPathDirectionPose(   const PoseStamped& global_pose, 
                                    const std::vector<PoseStamped>& plan,
                                    const int& n);
    Pose    getAimAtPathPose(       const PoseStamped& global_pose, 
                                    const std::vector<PoseStamped>& plan,
                                    const int& n);

    VelCalcResult calculateDriveVelocityCommand(Twist& cmd_vel);
    VelCalcResult calculateRotateVelocityCommand(Twist& cmd_vel);
    VelCalcResult calculateStrafeVelocityCommand(Twist& cmd_vel);
    float   radiusFromVelocity(const Twist& velocity);
    float   currentRadius();
    Arc     createArcFromPoseAndTarget(const Pose& start_pose, const rose_geometry::Point& check_waypoint);
    Arc     createArcFromVelocity(const Pose& start_pose, const Twist& vel, const float& percentage_of_circle);
    
    std::vector<Position2DInt>               getLethalCellsInPolygon(const vector<rose_geometry::Point>& polygon);
    std::vector<rose_geometry::Point>        getCellsPoints(    const vector<Position2DInt>& cells, 
                                                                bool only_center_point, 
                                                                const vector<rose_geometry::Point>& filter_polygon = vector<rose_geometry::Point>());

    bool                    trajectoryCollidesWithCells(    TrajectoryFootprint& footprint_trajectory,
                                                            const vector<Position2DInt>& cells);
    bool                    trajectoryCollidesWithPoints(   TrajectoryFootprint& footprint_trajectory,
                                                            const vector<rose_geometry::Point>& points);

    string            name_;
    string            global_frame_;
    string            robot_base_frame_;
    bool              initialized_;
    ros::NodeHandle   pn_;


    tf::TransformListener*       tf_listener_;    
    costmap_2d::Costmap2DROS*    costmap_ros_;
    costmap_2d::Costmap2D*       costmap_;
    ArcLocalPlannerCostmapModel* world_model_;   ///< @brief The world model that the controller uses for collision detection


    vector<PoseStamped> global_plan_;
    vector<PoseStamped> transformed_plan_;

    PoseStamped         global_goal_;

    // Robot state
    PoseStamped         global_pose_;
    Twist               local_vel_;
    
    vector<rose_geometry::Point>       footprint_;
    vector<rose_geometry::Point>       transformed_footprint_;
    double              inscribed_radius_;
    double              circumscribed_radius_;

    Twist               last_set_cmd_vel_;

    ros::Publisher                  g_plan_pub_;
    ros::Publisher                  t_plan_pub_;
    ros::Publisher                  l_plan_pub_;
    ros::Publisher                  simulation_plan_pub_;    
    ros::Publisher                  rviz_markers_pub_;
    ros::Publisher                  rviz_marker_pub_;
    map<string, ros::Publisher>     footprint_pubs_;

    base_local_planner::LocalPlannerUtil    planner_util_;
    base_local_planner::FootprintHelper     footprint_helper_;
    base_local_planner::OdometryHelperRos   odom_helper_;

    StatemachineState state_;
    StatemachineState prev_state_;

    int start_index_;

    rose_geometry::Point steepest_descent_direction_;
    Pose state_start_pose_;
    Pose state_target_pose_;

    vector<Position2DInt>                       cur_footprint_cells_;
    std::map<std::pair<int,int>, unsigned int>  footprint_cells_multimap_;  
    int marker_list_id_;

    float path_direction_;
    bool found_valid_cmd_vel_;

    float prev_ranking_;
    int prev_index_;

    double distance_weight_; 
    double clearance_weight_;
    double difference_weight_;

    float distance_squared_prev_;
    float angle_difference_prev_;

    TwistMAF cmd_vel_maf_; 

    ros::Time begin_;

    FootprintCollisionChecker FCC_;

    Timing timing_a_;
};
};

#endif // ARC_LOCAL_PLANNER_H
