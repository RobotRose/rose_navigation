/***********************************************************************************
* Copyright: Rose B.V. (2014)
*
* Revision History:
*   Author: Okke Hendriks
*   Date  : 2014/03/19
*       - File created.
*
* Description:
*   Arc class
* 
***********************************************************************************/

#ifndef ARC_HPP
#define ARC_HPP

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

#include "rose_common/common.hpp"
#include "rose_geometry/point.hpp"
#include "rose_geometry/geometry.hpp"
#include "rose_conversions/conversions.hpp"

using geometry_msgs::Pose;

class Arc
{
  public:
    Arc()
        : signed_radius_(0.0)
        , abs_radius_(0.0)
        , start_angle_(0.0)
        , stop_angle_(0.0)
        , pointize_steps_(0)
        , start_tangent_angle_(0.0)
        , stop_tangent_angle_(0.0)
        , clockwise_(true)
        , extra_start_distance_(0.0)
        , extra_stop_distance_(0.0)
    {}

    // Create arc given start point, tangent angle and a target point
    Arc(const rose_geometry::Point& start_point, float tangent_angle, const rose_geometry::Point& target_point)
        : pointize_steps_(10)
        , start_point_(start_point)
        , stop_point_(target_point)
        , clockwise_(false)
        , extra_start_distance_(0.0)
        , extra_stop_distance_(0.0)
      {
        float dist                  = rose_geometry::distance(start_point, target_point);
        float angle_between_points  = angle(start_point, target_point);
        float angle_to_center       = 0.0;

        
        if(rose_geometry::getShortestSignedAngle(tangent_angle, angle_between_points) <= 0.0)
        {
            // CW turn
            angle_to_center = rose_geometry::getShortestSignedAngle(angle_between_points, tangent_angle - 0.5*M_PI);
            signed_radius_ = -(dist/2.0)/cos(angle_to_center);
            clockwise_ = true;
        }
        else
        {
            // CCW turn
            angle_to_center = rose_geometry::getShortestSignedAngle(angle_between_points, tangent_angle + 0.5*M_PI);
            signed_radius_ = (dist/2.0)/cos(angle_to_center);
            clockwise_ = false;
        }

        abs_radius_ = fabs(signed_radius_);

        //! @todo OH: add check for tangent_angle->angle_between_points == -0.5*M_PI, 0.0, +0.5*M_PI
        
        center_.x   = target_point.x - start_point.x;
        center_.y   = target_point.y - start_point.y;
        rose_geometry::rotateVect(&center_.x, &center_.y, angle_to_center);
        rose_geometry::setVectorLengthXY(&center_.x, &center_.y, abs_radius_);
        center_.x   += start_point.x;
        center_.y   += start_point.y;

        start_angle_    = angle(center_, start_point);
        stop_angle_     = angle(center_, target_point);

        float shortest_angle = rose_geometry::getShortestSignedAngle(start_angle_, stop_angle_);

        if(shortest_angle <= 0.0)
        {
            // Shortest rotation is CW (shortest_angle is negative)
            if(clockwise_)
                stop_angle_     = start_angle_ + shortest_angle;
            else
                stop_angle_     = start_angle_ + (2.0*M_PI + shortest_angle);
        }
        else
        {
            // Shortest rotation is CCW (shortest_angle is positive)
            if(clockwise_)
                stop_angle_     = start_angle_ + (-2.0*M_PI + shortest_angle);
            else
                stop_angle_     = start_angle_ + shortest_angle;
        }

        recalculateArcPoses();
    }

    const rose_geometry::Point& getStartPoint() const
    {
        return start_point_;
    }

    const rose_geometry::Point& getStartPointExtra() const
    {
        return start_point_;
    }

    const rose_geometry::Point& getStopPoint() const
    {
        return stop_point_; 
    }

    const rose_geometry::Point& getStopPointExtra() const
    {
        return stop_point_; 
    }

    const rose_geometry::Point& getCenter() const
    {
        return center_;
    }

    void setCenter(const rose_geometry::Point& center)
    {
        if(center.x != center_.x || center.y != center_.y)
        {
            center_ = center;
            recalculateArcPoses();
        }
    }

    float getSignedRadius() const
    {
        return signed_radius_;
    }

    void setSignedRadius(float radius)
    {
        if(radius != signed_radius_)
        {
            signed_radius_ = radius;
            abs_radius_    = fabs(signed_radius_);
            recalculateArcPoses();
        }
    }

    float getAbsRadius() const
    {
        return abs_radius_;
    }

    float getStartAngle() const
    {
        return start_angle_;
    }

    float getStartAngleExtra() const
    {
        return start_angle_ + extra_start_distance_/getSignedRadius();
    }

    void setStartAngle(float start_angle)
    {
        if(start_angle != start_angle_)
        {
            start_angle_ = start_angle;
            recalculateArcPoses();
        }
    }

    float getStopAngle() const
    {
        return stop_angle_;
    }

    float getStopAngleExtra() const
    {
        return stop_angle_ - extra_start_distance_/getSignedRadius();
    }

    void setStopAngle(float stop_angle)
    {
        if(stop_angle != stop_angle_)
        {
            stop_angle_ = stop_angle;
            recalculateArcPoses();
        }
    }

    Pose getArcPose(float angle) const
    {
        Pose arc_pose; 

        float current_angle = angle;
        arc_pose.position.x = abs_radius_;
        arc_pose.position.y = 0.0;
        rose_geometry::rotateVect(&arc_pose.position.x, &arc_pose.position.y, current_angle);
        arc_pose.position.x += center_.x;
        arc_pose.position.y += center_.y;
        arc_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, getTangentAngle(current_angle));

        return arc_pose;
    }

    Pose getStartPose() const
    {
        return getArcPose(getStartAngle());
    }

    Pose getStartPoseExtra() const
    {
        return getArcPose(getStartAngleExtra());
    }

    Pose getStopPose() const
    {
        return getArcPose(getStopAngle());
    }

    Pose getStopPoseExtra() const
    {
        return getArcPose(getStopAngleExtra());
    }


    const std::vector<Pose>& getPoses() const
    {
        return arc_poses_;
    }

    const std::vector<Pose>& getExtraPoses() const
    {
        return arc_poses_;
    }

    float getAngleMeasure() const
    {
        return fabs(getStopAngle() - getStartAngle());
    }

    float getAngleMeasureExtra() const
    {
        return fabs(getStopAngleExtra() - getStartAngleExtra());
    }

    float getLength() const
    {
        return fabs(getAngleMeasure()*getSignedRadius());
    }

    float getLengthExtra() const
    {
        return fabs(getAngleMeasureExtra()*getSignedRadius());
    }

    float getLength(float angle_measure) const
    {
        return fabs(angle_measure*getSignedRadius());
    }

    float getStartTangentAngle() const
    {
        return getTangentAngle(getStartAngle());
    }

    float getStartTangentAngleExtra() const
    {
        return getTangentAngle(getStartAngleExtra());
    }

    float getStopTangentAngle() const
    {
        return getTangentAngle(getStopAngle());
    }

    float getStopTangentAngleExtra() const
    {
        return getTangentAngle(getStopAngleExtra());
    }

    float getTangentAngle(float angle) const
    {
        return angle + rose_conversions::sgn(stop_angle_ - start_angle_)*0.5*M_PI;
    }

    bool isCW() const
    {
        return clockwise_;
    }

    bool isCCW() const
    {
        return !clockwise_;
    }

    float getExtraStartDistance() const
    {
        return extra_start_distance_;
    }

    void setExtraStartDistance(float dist)
    {
        if(dist != extra_start_distance_)
        {
            extra_start_distance_ = dist;
            recalculateArcPoses();
        }
    }

    float getExtraStopDistance() const
    {
        return extra_stop_distance_;
    }

    void setExtraStopDistance(float dist)
    {
        if(dist != extra_stop_distance_)
        {
            extra_stop_distance_ = dist;
            recalculateArcPoses();
        }
    }


    // Get CCW polygon of a minimal square around the arc, taking into account the margin.
    std::vector<rose_geometry::Point> getBoundingSquare(float margin) const
    {
        rose_geometry::Point point;
        std::vector<rose_geometry::Point> bounding_square;

        ROS_DEBUG_NAMED(ROS_NAME, "Change this, this does not work with all start_angles!");
        float x_max = std::max(start_point_.x, stop_point_.x) + margin;
        float y_max = std::max(start_point_.y, stop_point_.y) + margin;
        float x_min = std::min(start_point_.x, stop_point_.x) - margin;
        float y_min = std::min(start_point_.y, stop_point_.y) - margin;
        
        point.x = x_max;
        point.y = y_max;
        bounding_square.push_back(point);
        point.x = x_min;
        point.y = y_max;
        bounding_square.push_back(point);
        point.x = x_min;
        point.y = y_min;
        bounding_square.push_back(point);
        point.x = x_max;
        point.y = y_min;
        bounding_square.push_back(point);

        return bounding_square;
    }


    std::vector<rose_geometry::Point> getArcPoints(float resolution)
    {
        return getArcPoints(getStartAngle(), getStopAngle(), resolution);
    }

    std::vector<rose_geometry::Point> getArcPoints(float start_angle, float stop_angle, float resolution)
    {
        rose_geometry::Point                arc_point;       
        std::vector<rose_geometry::Point>   arc_points;

        // Simply return the center point if the radius is zero
        if(getAbsRadius() == 0.0)
        {
            arc_points.push_back(getCenter());
            return arc_points;
        }

        // Minimal 2 points maximally 100 !//! @todo magic numbers
        int n = (int)fmax(2.0, fmin(100.0, (int)ceil((getLength(fabs(start_angle - stop_angle))/resolution))));

        float w_step    = (stop_angle - start_angle)/((float)n);
        int   step      = 0;
        for(step; step <= n; step++)
        {
            float current_angle = start_angle + (float)step * w_step;
            arc_point.x = abs_radius_;
            arc_point.y = 0.0;
            rose_geometry::rotateVect(&arc_point.x, &arc_point.y, current_angle);
            arc_point.x += center_.x;
            arc_point.y += center_.y;

            arc_points.push_back(arc_point);
        }

        return arc_points;
    }

  private:
    rose_geometry::Point  center_;
    rose_geometry::Point  start_point_;
    rose_geometry::Point  stop_point_;
    float                           signed_radius_;
    float                           abs_radius_;
    float                           start_angle_;
    float                           stop_angle_;
    float                           start_tangent_angle_;
    float                           stop_tangent_angle_;
    float                           extra_start_distance_;
    float                           extra_stop_distance_;
    int                             pointize_steps_;
    bool                            clockwise_;

    std::vector<Pose>                arc_poses_;

    void recalculateArcPoses()
    {
        // Calculate n points on the arc
        arc_poses_.clear();

        // Minimal 2 points maximally 25 !//! @todo magic numbers
        pointize_steps_ = (int)fmax(2.0, fmin(25.0, (int)ceil((getLength()/0.01))));

        if(pointize_steps_ <= 0)
            return;

        Pose arc_pose;      

        float w_step    = (getStopAngleExtra() - getStartAngleExtra())/((float)pointize_steps_);
        int   step      = 0;
        for(step; step <= pointize_steps_; step++)
        {
            float current_angle = getStartAngleExtra() + (float)step * w_step;
            arc_pose.position.x = abs_radius_;
            arc_pose.position.y = 0.0;
            rose_geometry::rotateVect(&arc_pose.position.x, &arc_pose.position.y, current_angle);
            arc_pose.position.x += center_.x;
            arc_pose.position.y += center_.y;
            arc_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, getTangentAngle(current_angle));
            arc_poses_.push_back(arc_pose);
        }
    }

};

#endif // #define ARC_HPP
