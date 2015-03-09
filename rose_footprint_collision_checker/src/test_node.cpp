#include "polyclipping/clipper.hpp"

#include <stdio.h>      /* printf, scanf, puts, NULL */
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */
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

#define POLYGON_PRECISION 1000000.0

using namespace ClipperLib;

typedef rose_geometry::Point                    Vertex;
typedef rose_geometry::Stamped<Vertex>          StampedVertex;
typedef std::vector<Vertex>                     Vertices;
typedef std::vector<StampedVertex>              StampedVertices;
typedef std::vector<Vertex>                     Polygon;
typedef std::vector<Polygon>                    Polygons;
typedef std::vector<geometry_msgs::PoseStamped> Trajectory;

struct Timing
{
    std::vector<ros::Time> start_times;
    std::vector<ros::Time> finish_times;
};

Path polygonToPath(const Polygon& polygon)
{
    Path path;
    for(const Vertex& vertex : polygon)
        path.push_back( IntPoint((cInt)(vertex.x*POLYGON_PRECISION), (cInt)(vertex.y*POLYGON_PRECISION)) );
    return path;
}

Paths polygonsToPaths(const Polygons& polygons)
{
    Paths paths;
    for(const Polygon& polygon : polygons)
        paths.push_back(polygonToPath(polygon));
    return paths;
}

Polygon pathToPolygon(const Path& path)
{
    Polygon polygon;
    for(const auto& int_point : path)
        polygon.push_back( rose_geometry::Point((float)(int_point.X)/POLYGON_PRECISION, (float)(int_point.Y)/POLYGON_PRECISION, 0.0) );
    return polygon;
}

Polygons pathsToPolygons(const Paths& paths)
{
    Polygons polygons;
    for(const Path& path : paths)
        polygons.push_back(pathToPolygon(path));
    return polygons;
}

Polygon createAABB( const Polygon& polygon,
                    float margin)
{
    float minx = 1e6;
    float maxx = -1e6;
    float miny = 1e6;
    float maxy = -1e6;
    for(const auto& point : polygon)
    {
        minx = fmin(point.x, minx);
        maxx = fmax(point.x, maxx);
        miny = fmin(point.y, miny);
        maxy = fmax(point.y, maxy);
    }

    minx -= margin;
    maxx += margin;
    miny -= margin;
    maxy += margin;

    vector<rose_geometry::Point> bounding_polygon;
    bounding_polygon.push_back(Vertex(maxx, maxy, 0.0)); 
    bounding_polygon.push_back(Vertex(minx, maxy, 0.0));
    bounding_polygon.push_back(Vertex(minx, miny, 0.0));
    bounding_polygon.push_back(Vertex(maxx, miny, 0.0));

    return bounding_polygon;
}

bool inAABB(const Vertex& point, const Polygon& aabb)
{
    if(point.x < aabb.at(0).x and point.x > aabb.at(2).x and point.y < aabb.at(0).y and point.y >  aabb.at(2).y)
        return true;

    return false;
}

int collision(const Polygon& polygon, const StampedVertices& stamped_lethal_points)
{
    if(stamped_lethal_points.empty())
        return false;

    Path path = polygonToPath(polygon);
    int id = 0;
    int collisions = 0;
    ROS_INFO("Checking for collision using %d points and a polygon with %d vertices.", (int)stamped_lethal_points.size(), (int)path.size());
    Polygon aabb = createAABB(polygon, 0.001);
    for(const auto& stamped_lethal_point : stamped_lethal_points)
    {
        if(inAABB(stamped_lethal_point.data, aabb))
        {
            if(PointInPolygon(IntPoint(stamped_lethal_point.data.x*POLYGON_PRECISION, stamped_lethal_point.data.y*POLYGON_PRECISION), path))
                collisions++;
            continue;
        }
    }

    return collisions;
}

void processTiming(const std::string& name, Timing& time_struct, const int& average_over, const float& rate)
{
    float sum = 0;
    int samples = time_struct.start_times.size();
    for(int i = 0; i < samples; i++)
    {
        sum += time_struct.finish_times.at(i).toSec() - time_struct.start_times.at(i).toSec();
    }

    sum /= (float)samples;

    if(samples - average_over - 1 > 0)
    {
        time_struct.start_times.erase(time_struct.start_times.begin(), std::next(time_struct.start_times.begin(), (samples - average_over - 1) ));
        time_struct.finish_times.erase(time_struct.finish_times.begin(), std::next(time_struct.finish_times.begin(), (samples - average_over - 1) ));
    }

    ROS_INFO_THROTTLE_NAMED(rate, ROS_NAME, "Timing '%s', timing over %d samples, average: %.6f", name.c_str(), samples, sum);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "move_base_smc");
    ros::NodeHandle n;

    srand (time(NULL));

    Timing time_a;

    for(int j = 0; j < 100; j++)
    {
        Polygon test_poly;
        StampedVertices lethal_points;

        std_msgs::Header header;
        header.stamp = ros::Time::now();
        header.frame_id = "map";

        test_poly.push_back(Vertex(1000, 0, 0.0));
        for(int i = 0; i < 5000; i++)
            test_poly.push_back(Vertex(rand() % 1000 + 1 , rand() % 1000 + 1 , 0.0));

        for(int i = 0; i < 20000; i++)
            lethal_points.push_back(StampedVertex(header, Vertex(rand() % 1000 + 1, rand() % 1000 + 1, 0.0)));

        time_a.start_times.push_back(ros::Time::now());

        int collisions = collision(test_poly, lethal_points);

        ROS_INFO("Collisions %d", collisions);

        time_a.finish_times.push_back(ros::Time::now());
        processTiming("time_a", time_a, 100, 0.2);

        if(not n.ok())
            return 1;
    }

    ros::Duration(5.0).sleep();
    processTiming("time_a", time_a, 1, 0.2);

    return 0;
}
