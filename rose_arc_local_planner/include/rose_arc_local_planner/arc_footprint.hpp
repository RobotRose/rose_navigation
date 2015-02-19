/***********************************************************************************
* Copyright: Rose B.V. (2014)
*
* Revision History:
*   Author: Okke Hendriks
*   Date  : 2014/07/09
*       - File created.
*
* Description:
*   Super class of TrajectoryFootprint generates the trajectory footprint of an arc.
* 
***********************************************************************************/

#ifndef ARC_FOOTPRINT_HPP
#define ARC_FOOTPRINT_HPP

#include "boost/foreach.hpp"
#include "rose_common/common.hpp"
#include "rose_geometry/point.hpp"
#include "rose_arc_local_planner/arc.hpp"
#include "rose_arc_local_planner/trajectory_footprint.hpp"

class ArcFootprint : public TrajectoryFootprint
{
  public: 
    ArcFootprint(const Arc& arc, const vector<rose_geometry::Point>& footprint);
    ArcFootprint(const Arc& arc, const vector<rose_geometry::Point>& footprint, float resolution);
    bool initialize(const Arc& arc, const vector<rose_geometry::Point>& footprint, float resolution);
    ~ArcFootprint();

    Arc getArc() const;
    const Arc& getArcRef() const;

    rose_geometry::Point getEndPoint() const;

    int global_path_index_;

  private:
    bool generatePolygon();

    Arc                                     arc_;
    vector<rose_geometry::Point>  footprint_;
    float                                   resolution_;
};

#endif // ARC_FOOTPRINT_HPP
