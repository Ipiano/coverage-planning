#pragma once

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/multi/geometries/multi_linestring.hpp>

#include <boost/units/systems/si.hpp>
#include <boost/units/systems/angle/degrees.hpp>

namespace ads
{
namespace ccpp
{
namespace geometry
{
typedef boost::geometry::model::d2::point_xy<double> Point2d;

typedef boost::geometry::model::polygon<Point2d> Polygon2d;
typedef boost::geometry::ring_type<Polygon2d>::type Ring2d;
typedef boost::geometry::model::box<Point2d> Box2d;

typedef boost::geometry::model::linestring<Point2d> Line2d;
typedef boost::geometry::model::multi_linestring<Line2d> MultiLine2d;

typedef boost::geometry::model::segment<Point2d> Segment2d;
typedef boost::geometry::model::referring_segment<Point2d> ReferringSegment2d;
typedef boost::geometry::model::referring_segment<const Point2d> ConstReferringSegment2d;
}

namespace quantity
{
typedef boost::units::quantity<boost::units::si::plane_angle> Radians;
typedef boost::units::quantity<boost::units::degree::plane_angle> Degrees;
}

namespace units
{
const auto Radian = boost::units::si::radian;
const auto Degree = boost::units::degree::degree;
}
}
}
