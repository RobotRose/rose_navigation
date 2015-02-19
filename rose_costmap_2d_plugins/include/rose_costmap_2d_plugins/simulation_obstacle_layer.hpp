#ifndef SIMULATION_OBSTACLE_COSTMAP_PLUGIN_H_
#define SIMULATION_OBSTACLE_COSTMAP_PLUGIN_H_

#include <vector>
#include <utility>
#include <algorithm>
#include <boost/foreach.hpp>

#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/costmap_layer.h>
#include <costmap_2d/footprint_layer.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>

#include "ros_name/ros_name.hpp"

#include "rose_conversions/conversions.hpp"

namespace rose_costmap_2d_plugins
{

using costmap_2d::NO_INFORMATION;
using costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::FREE_SPACE;

class SimulationObstacleLayer : public costmap_2d::CostmapLayer
{
public:
    SimulationObstacleLayer();
    ~SimulationObstacleLayer();


    virtual void onInitialize();
    virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x,
                             double* max_y);
    virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

    virtual void activate();
    virtual void deactivate();
    virtual void reset();

    void setResetBounds(double mx0, double mx1, double my0, double my1)
    {
        reset_min_x_ = std::min(mx0, reset_min_x_);
        reset_max_x_ = std::max(mx1, reset_max_x_);
        reset_min_y_ = std::min(my0, reset_min_y_);
        reset_max_y_ = std::max(my1, reset_max_y_);
        has_been_reset_ = true;
    }

  
protected:
    std::string         global_frame_;          ///< @brief The global frame for the costmap
    ros::Subscriber     rviz_clicked_point_sub_;
    bool                rolling_window_;
    bool                use_maximum_;
    bool                new_click_;
    bool                has_been_reset_;

    double reset_min_x_;
    double reset_min_y_;
    double reset_max_x_;
    double reset_max_y_;

    costmap_2d::FootprintLayer          footprint_layer_;            ///< @brief clears the footprint in this obstacle layer.

    /** @brief Overridden from superclass Layer to pass new footprint into footprint_layer_. */
    virtual void onFootprintChanged();
  
private:
    ros::NodeHandle             n_;
    double                      min_x_, min_y_, max_x_, max_y_;
    
    std::vector<geometry_msgs::PointStamped>    clicked_points_;
    geometry_msgs::PointStamped                 last_clicked_point_;

    dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;

    void CB_reconfigure(costmap_2d::GenericPluginConfig &config, uint32_t level);
    void CB_rviz_clicked_point(const geometry_msgs::PointStamped& clicked_point);
};

}

#endif // SIMULATION_OBSTACLE_COSTMAP_PLUGIN_H_
