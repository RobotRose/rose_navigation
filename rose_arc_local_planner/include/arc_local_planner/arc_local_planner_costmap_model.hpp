/***********************************************************************************
* Copyright: Rose B.V. (2014)
*
* Revision History:
*	Author: Okke Hendriks
*	Date  : 2014/03/12
* 		- File created.
*
* Description:
*	Costmap model which provides acces to ros its standard lineCost function
* 
***********************************************************************************/

#ifndef ARC_LOCAL_PLANNER_COSTMAP_MODEL 
#define ARC_LOCAL_PLANNER_COSTMAP_MODEL 

#include <ros/ros.h>

#include <tf/transform_datatypes.h>

#include <base_local_planner/world_model.h>
#include <base_local_planner/footprint_helper.h>
#include <base_local_planner/Position2DInt.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/cost_values.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>

#include <boost/foreach.hpp>

#include "rose_common/common.hpp"

using namespace std;
using namespace costmap_2d;

namespace rose_navigation {

class ArcLocalPlannerCostmapModel : public base_local_planner::WorldModel
{
  public:
    ArcLocalPlannerCostmapModel(const Costmap2D& costmap);
    virtual ~ArcLocalPlannerCostmapModel();


    vector<base_local_planner::Position2DInt> getFootprintCells(const geometry_msgs::Pose pose, const std::vector<geometry_msgs::Point>& footprint, bool fill);

    double footprintCost(   const geometry_msgs::Point& position, 
                            const vector<geometry_msgs::Point>& footprint, 
                            double inscribed_radius, 
                            double circumscribed_radius);

    unsigned int footprintCost(     const geometry_msgs::Pose& pose, 
                                    const vector<geometry_msgs::Point>& footprint, 
                                    double inscribed_radius, 
                                    double circumscribed_radius);

    unsigned int cellsCost( const vector<base_local_planner::Position2DInt> cells, 
                            float& summed_cost);

    unsigned int footprintCostExcluding(    const geometry_msgs::Pose& test_pose, 
                                            const vector<geometry_msgs::Point>& footprint, 
                                            double inscribed_radius, 
                                            double circumscribed_radius, 
                                            const std::map<std::pair<int,int>, unsigned int>& cur_footprint_cells);

    unsigned int footprintCostExcluding(    const geometry_msgs::Pose& test_pose, 
                                            const vector<geometry_msgs::Point>& footprint, 
                                            double inscribed_radius, 
                                            double circumscribed_radius, 
                                            const std::map<std::pair<int,int>, unsigned int>& cur_footprint_cells,
                                            float& summed_cost);
    
    double lineCost(int x0, int y0, int x1, int y1, std::vector<base_local_planner::Position2DInt>& points);
    double pointCost(int x, int y);

  private:
    	const Costmap2D& 						costmap_; 

    	ros::Publisher      					rviz_markers_pub_;

    	base_local_planner::FootprintHelper     footprint_helper_;
};

};

#endif // ARC_LOCAL_PLANNER_COSTMAP_MODEL 
