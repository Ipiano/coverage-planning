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

inline double sqmag(const Point2d& p)
{
    return p.x() * p.x() + p.y() * p.y();
}

inline double sqmag(const Point2d& p1, const Point2d& p2)
{
    Point2d diff = p2;
    boost::geometry::subtract_point(diff, p1);

    return sqmag(diff);
}

inline bool haveSameXCoord(const Point2d& p1, const Point2d& p2)
{
    return std::abs(p1.x() - p2.x()) < epsilon;
}

inline bool equal(const Point2d& p1, const Point2d& p2)
{
    constexpr double epsilonSq = epsilon * epsilon;

    return sqmag(p1, p2) < epsilonSq;
}

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
