#include "rose_arc_local_planner/arc_local_planner_costmap_model.hpp"

namespace rose_navigation {

ArcLocalPlannerCostmapModel::ArcLocalPlannerCostmapModel(const Costmap2D& costmap)
	: costmap_(costmap)
{}

ArcLocalPlannerCostmapModel::~ArcLocalPlannerCostmapModel()
{}

vector<base_local_planner::Position2DInt> ArcLocalPlannerCostmapModel::getFootprintCells(const geometry_msgs::Pose pose, const std::vector<geometry_msgs::Point>& footprint, bool fill)
{
    Eigen::Vector3f pos(pose.position.x, pose.position.y, tf::getYaw(pose.orientation));
    return footprint_helper_.getFootprintCells(pos, footprint, costmap_, fill);
}

double ArcLocalPlannerCostmapModel::footprintCost(const geometry_msgs::Point& position, const vector<geometry_msgs::Point>& footprint, double inscribed_radius, double circumscribed_radius)
{
    ROS_ASSERT_MSG(true, "double ArcLocalPlannerCostmapModel::footprintCost -> Do not use this function!");
}

unsigned int ArcLocalPlannerCostmapModel::footprintCost(const geometry_msgs::Pose& test_pose, const vector<geometry_msgs::Point>& footprint, double inscribed_radius, double circumscribed_radius)
{
    Eigen::Vector3f test_pos(test_pose.position.x, test_pose.position.y, tf::getYaw(test_pose.orientation));
    vector<base_local_planner::Position2DInt> test_footprint_cells = footprint_helper_.getFootprintCells(test_pos, footprint, costmap_, false);

    unsigned int max_cost = 0;
    
    for(auto cell : test_footprint_cells)
    {
        unsigned int cost = costmap_.getCost(cell.x, cell.y);
        if(cost > max_cost)
        {
            max_cost = cost;

            if(max_cost == costmap_2d::LETHAL_OBSTACLE)
                return costmap_2d::LETHAL_OBSTACLE;
        }
    }

    return max_cost;
}

unsigned int ArcLocalPlannerCostmapModel::cellsCost(    const vector<base_local_planner::Position2DInt> cells, 
                                                        float& summed_cost)
{
    unsigned int max_cost   = 0;  
    int number_of_cells     = 0;
    for(auto cell : cells)
    {
        unsigned int cost   = costmap_.getCost(cell.x, cell.y);
        summed_cost         += cost;
        number_of_cells++;

        if(cost > max_cost)
        {            
            max_cost = cost;
            if(cost == costmap_2d::LETHAL_OBSTACLE)
                break;
        }

    }

    summed_cost = (summed_cost/number_of_cells)/255.0;
    return max_cost;
}

unsigned int ArcLocalPlannerCostmapModel::footprintCostExcluding(const geometry_msgs::Pose& test_pose, const vector<geometry_msgs::Point>& footprint, double inscribed_radius, double circumscribed_radius, const std::map<std::pair<int,int>, unsigned int>& footprint_cells_multimap)
{
    float summed_cost;
    return footprintCostExcluding(test_pose, footprint, inscribed_radius, circumscribed_radius, footprint_cells_multimap, summed_cost);
}

unsigned int ArcLocalPlannerCostmapModel::footprintCostExcluding(const geometry_msgs::Pose& test_pose, const vector<geometry_msgs::Point>& footprint, double inscribed_radius, double circumscribed_radius, const std::map<std::pair<int,int>, unsigned int>& footprint_cells_multimap, float& summed_cost)
{
    Eigen::Vector3f test_pos(test_pose.position.x, test_pose.position.y, tf::getYaw(test_pose.orientation));
    vector<base_local_planner::Position2DInt> test_footprint_cells = footprint_helper_.getFootprintCells(test_pos, footprint, costmap_, true);

    unsigned int max_cost   = 0;  
    int number_of_cells     = 0;
    for(auto cell : test_footprint_cells)
    {
        unsigned int cost   = costmap_.getCost(cell.x, cell.y);
        summed_cost         += cost;
        number_of_cells++;

        if(cost > max_cost)
        {
            // Do not take into account cell if inside current footprint
            if(footprint_cells_multimap.find(make_pair(cell.x, cell.y)) == footprint_cells_multimap.end())
            {
                max_cost = cost;               
                if(cost == costmap_2d::LETHAL_OBSTACLE)
                {
                    summed_cost = (summed_cost/number_of_cells)/255.0;      // 255 is MAX cost, so scale to 0.0 -> 1.0
                    return costmap_2d::LETHAL_OBSTACLE;
                }
            }           
        }
    }

    summed_cost = (summed_cost/number_of_cells)/255.0;
    return max_cost;
}


double ArcLocalPlannerCostmapModel::lineCost(int x0, int y0, int x1, int y1, std::vector<base_local_planner::Position2DInt>& points)
{
    ROS_DEBUG_NAMED(ROS_NAME, "lineCost [%d, %d] -> [%d, %d]", x0, y0, x1, y1);
    //Bresenham Ray-Tracing
    int deltax = abs(x1 - x0);        // The difference between the x's
    int deltay = abs(y1 - y0);        // The difference between the y's
    int x = x0;                       // Start x off at the first pixel
    int y = y0;                       // Start y off at the first pixel
    int xinc1, xinc2, yinc1, yinc2;
    int den, num, numadd, numpixels;
    double line_cost = 0.0;
    double point_cost = -1.0;

    if (x1 >= x0)                 // The x-values are increasing
    {
        xinc1 = 1;
        xinc2 = 1;
    }
    else                          // The x-values are decreasing
    {
        xinc1 = -1;
        xinc2 = -1;
    }
    if (y1 >= y0)                 // The y-values are increasing
    {
        yinc1 = 1;
        yinc2 = 1;
    }
    else                          // The y-values are decreasing
    {
        yinc1 = -1;
        yinc2 = -1;
    }

    if (deltax >= deltay)         // There is at least one x-value for every y-value
    {
        xinc1 = 0;                  // Don't change the x when numerator >= denominator
        yinc2 = 0;                  // Don't change the y for every iteration
        den = deltax;
        num = deltax / 2;
        numadd = deltay;
        numpixels = deltax;         // There are more x-values than y-values
    }
    else                          // There is at least one y-value for every x-value
    {
        xinc2 = 0;                  // Don't change the x for every iteration
        yinc1 = 0;                  // Don't change the y when numerator >= denominator
        den = deltay;
        num = deltay / 2;
        numadd = deltax;
        numpixels = deltay;         // There are more y-values than x-values
    }

    points.clear();
    for (int curpixel = 0; curpixel <= numpixels; curpixel++)
    {
        point_cost = pointCost(x, y); //Score the current point
        
        base_local_planner::Position2DInt point;
        point.x = x;
        point.y = y;
        points.push_back(point);

        if(line_cost < point_cost)
            line_cost = point_cost;
        num += numadd;              // Increase the numerator by the top of the fraction
        if (num >= den)             // Check if numerator >= denominator
        {
            num -= den;               // Calculate the new numerator value
            x += xinc1;               // Change the x as appropriate
            y += yinc1;               // Change the y as appropriate
        }
        x += xinc2;                 // Change the x as appropriate
        y += yinc2;                 // Change the y as appropriate
    }

    return line_cost;
}

double ArcLocalPlannerCostmapModel::pointCost(int x, int y)
{
    unsigned char cost = costmap_.getCost(x, y);
    return (double)cost;
}

};