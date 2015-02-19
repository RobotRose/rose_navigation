/**********************************************************************************
* Copyright: Rose B.V. (2014)
*
* Revision History:
*   Author: Okke Hendriks
*   Date  : 2014/07/09
*       - File created.
*
* Description:
*   Base class for trajectory footprints
* 
**********************************************************************************/

#ifndef TRAJECTORY_FOOTPRINT_HPP
#define TRAJECTORY_FOOTPRINT_HPP

#include <ros/ros.h>
#include <vector>

#include "rose_geometry/geometry.hpp"
#include "ros_name/ros_name.hpp"

using std::vector;

class TrajectoryFootprint
{
  public:
    TrajectoryFootprint();
    ~TrajectoryFootprint();

    void                                            setCost(float cost);
    float                                           getCost() const;
    void                                            setGlobalPathDistance(float distance);
    float                                           getGlobalPathDistance() const;
    void                                            setPolygon(const vector<rose_geometry::Point>& polygon);
    vector<rose_geometry::Point>          getPolygon() const;
    const vector<rose_geometry::Point>&   getPolygonRef() const;
    bool                                            collides() const;
    void                                            collides(bool collides);
    virtual rose_geometry::Point          getEndPoint() const;

  protected:
    float                                       cost_;
    float                                       global_path_distance_;
    vector<rose_geometry::Point>      polygon_;
    bool                                        collides_;

  private:

};

#endif // TRAJECTORY_FOOTPRINT_HPP
