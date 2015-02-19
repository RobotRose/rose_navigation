
#include <rose_costmap_2d_plugins/simulation_obstacle_layer.hpp>

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(rose_costmap_2d_plugins::SimulationObstacleLayer, costmap_2d::Layer)

namespace rose_costmap_2d_plugins
{

SimulationObstacleLayer::SimulationObstacleLayer()
{
    costmap_            = NULL; // this is the unsigned char* member of parent class Costmap2D.
    rolling_window_     = false;
    use_maximum_        = false;
    new_click_          = false;
    has_been_reset_     = false;


    min_x_ = 0.0;
    min_y_ = 0.0;
    max_x_ = 0.0;
    max_y_ = 0.0;
}

SimulationObstacleLayer::~SimulationObstacleLayer()
{}

void SimulationObstacleLayer::onInitialize()
{
    ros::NodeHandle n_;
    ros::NodeHandle nh("~/" + name_);
    rolling_window_ = layered_costmap_->isRolling();
    current_ = true;
    matchSize();

    dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
    dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(&SimulationObstacleLayer::CB_reconfigure, this, _1, _2);
    dsrv_->setCallback(cb);

    global_frame_ = layered_costmap_->getGlobalFrameID();

    activate();

    //footprint_layer_.initialize(layered_costmap_, name_ + "_footprint", tf_);

    enabled_ = true;
}

void SimulationObstacleLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x, double* max_y)
{
    if (rolling_window_)
        updateOrigin(robot_x - getSizeInMetersX() / 2.0, robot_y - getSizeInMetersY() / 2.0);

    if (!enabled_)
        return;

    if (has_been_reset_)
    {
        *min_x = std::min(reset_min_x_, *min_x);
        *min_y = std::min(reset_min_y_, *min_y);
        *max_x = std::max(reset_max_x_, *max_x);
        *max_y = std::max(reset_max_y_, *max_y);
        reset_min_x_ = 1e6;
        reset_min_y_ = 1e6;
        reset_max_x_ = -1e6;
        reset_max_y_ = -1e6;
        has_been_reset_ = false;
    }

    *min_x = std::min(min_x_ - 1.0, *min_x);
    *min_y = std::min(min_y_ - 1.0, *min_y);
    *max_x = std::max(max_x_ + 1.0, *max_x);
    *max_y = std::max(max_y_ + 1.0, *max_y);

    ROS_DEBUG_NAMED(ROS_NAME, "AfterupdateBounds() '%s' ->   [%.2f, %.2f, %.2f, %.2f]", getName().c_str(), *min_x, *min_y, *max_x, *max_y);


    // footprint_layer_.updateBounds(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);
}

void SimulationObstacleLayer::updateCosts(costmap_2d::Costmap2D& master_costmap, int min_i, int min_j, int max_i, int max_j)
{
    if(!enabled_ )
        return;

    ROS_DEBUG_NAMED(ROS_NAME, "updateCosts() '%s'.", getName().c_str());


    unsigned int mx, my;

    if(new_click_)
    {
        // Look if any of the previously clicked points are closer than the costmap resolution
        auto found_point = std::find_if(clicked_points_.begin(), clicked_points_.end(),
            [this, last_clicked_point_](const geometry_msgs::PointStamped& point)
            {
                if(rose_geometry::distanceXY(point.point, last_clicked_point_.point) < 0.05)
                { 
                    return true;
                }

                return false;
            } 
            );

        if(found_point == clicked_points_.end())    
        {
            ROS_INFO_NAMED(ROS_NAME, "Adding point to costmap '%s'.", getName().c_str());
            clicked_points_.push_back(last_clicked_point_);
            // Update bounds
            BOOST_FOREACH(const geometry_msgs::PointStamped& point, clicked_points_)
            {
                min_x_ = std::min(min_x_, point.point.x);
                min_y_ = std::min(min_y_, point.point.y);
                max_x_ = std::max(max_x_, point.point.x);
                max_y_ = std::max(max_y_, point.point.y);        
            }

        }
        else
        {
            ROS_INFO_NAMED(ROS_NAME, "Erasing existing point from costmap '%s'.", getName().c_str());
            clicked_points_.erase(found_point);

            if(!master_costmap.worldToMap(found_point->point.x, found_point->point.y, mx, my))
                ROS_WARN_THROTTLE_NAMED(1.0, ROS_NAME, "Unable to transform clicked point [%.2f, %.2f] in frame: %s, to map coordinates of costmap '%s'.", found_point->point.x, found_point->point.y, found_point->header.frame_id.c_str(), getName().c_str());  
            else
            {
                master_costmap.setCost(mx, my, FREE_SPACE);
            }
        }

        new_click_ = false;
    }



    BOOST_FOREACH(const geometry_msgs::PointStamped& point, clicked_points_)
    {
        
        if(!master_costmap.worldToMap(point.point.x, point.point.y, mx, my))
        {
            ROS_WARN_THROTTLE_NAMED(1.0, ROS_NAME, "Origin [%.2f, %.2f],  mx,my [%d, %d], Resolution: %.2f", master_costmap.getOriginX(), master_costmap.getOriginY(), mx, my, master_costmap.getResolution());
            ROS_WARN_THROTTLE_NAMED(1.0, ROS_NAME, "Error while mapping clicked point [%.2f, %.2f] in frame: %s, to coordinates of costmap '%s'.", point.point.x, point.point.y, point.header.frame_id.c_str(), getName().c_str());  
        }
        else
        {
            master_costmap.setCost(mx, my, LETHAL_OBSTACLE);
        } 
    }

    
}

void SimulationObstacleLayer::activate()
{
    rviz_clicked_point_sub_ = n_.subscribe("/clicked_point", 1, &SimulationObstacleLayer::CB_rviz_clicked_point, this);

}
void SimulationObstacleLayer::deactivate()
{
    rviz_clicked_point_sub_.shutdown();
}

void SimulationObstacleLayer::reset()
{
    // Do not clear added obstacles
    // deactivate();
    // resetMaps();
    // clicked_points_.clear();      
    new_click_ = true;           // Re add the data

    // min_x_ = 0.0;
    // min_y_ = 0.0;
    // max_x_ = 0.0;
    // max_y_ = 0.0;
    //activate();
}

void SimulationObstacleLayer::CB_reconfigure(costmap_2d::GenericPluginConfig &config, uint32_t level)
{
    enabled_ = config.enabled;

    if(!enabled_)
        deactivate();
    else
        activate();
}

void SimulationObstacleLayer::CB_rviz_clicked_point(const geometry_msgs::PointStamped& clicked_point)
{
    ROS_INFO_NAMED(ROS_NAME, "You clicked on a point in rviz [%.2f, %.2f] in frame: %s.", clicked_point.point.x, clicked_point.point.y, clicked_point.header.frame_id.c_str());

    last_clicked_point_ = clicked_point;

    new_click_ = true;

}



void SimulationObstacleLayer::onFootprintChanged()
{
    footprint_layer_.onFootprintChanged();
}

} // end namespace rose_costmap_2d_plugins
