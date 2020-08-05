#pragma once

#include "ads/epsilon.h"

#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/arithmetic/arithmetic.hpp>

namespace ads
{
namespace algorithms
{
namespace sweep_line
{
typedef boost::geometry::model::d2::point_xy<double> Point2d;
typedef boost::geometry::model::polygon<Point2d> Polygon2d;

//! Calculates the square magnitude of the line from (0, 0) to the given point
//! \param p Point to calculate for
//! \return Square magnitude
inline double sqmag(const Point2d& p)
{
    return p.x() * p.x() + p.y() * p.y();
}

//! Calculates the square magnitude of the line between two points
//! \param p1 First endpoint
//! \param p2 Second endpoint
//! \return Square magnitude
inline double sqmag(const Point2d& p1, const Point2d& p2)
{
    Point2d diff = p2;
    boost::geometry::subtract_point(diff, p1);

    return sqmag(diff);
}

//! Checks if two points have the same X coordinate (within epsilon)
//! Uses global epsilon value
//! \param p1 First point
//! \param p2 Second point
//! \return true if p1 and p2 have X coordinates with epsilon of each other
inline bool haveSameXCoord(const Point2d& p1, const Point2d& p2)
{
    return std::abs(p1.x() - p2.x()) < epsilon;
}

//! Checks if two points are within epsilon distance of each other
//! \param p1 First point
//! \param p2 Second point
//! \return true if p1 and p2 are within epsilon of each other
inline bool equal(const Point2d& p1, const Point2d& p2)
{
    constexpr double epsilonSq = epsilon * epsilon;

    return sqmag(p1, p2) < epsilonSq;
}

//! Sorts points for sweep-line ordering
//! This is left-to-right, bottom-to-top
//! \param left First point to compare
//! \param right Second point to compare
//! \return true if left is before right in sort order
inline bool pointLessThan(const Point2d& left, const Point2d& right)
{
    if (left.x() < right.x())
        return true;
    else if (left.x() > right.x())
        return false;
    else
        return left.y() < right.y();
};
}
}
}
