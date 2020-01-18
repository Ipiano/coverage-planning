#pragma once

#include <boost/geometry/core/cs.hpp>

#include "ads/ccpp/typedefs.h"

namespace ads
{
namespace ccpp
{
namespace desktop_tool
{
namespace geometry
{
template <class BoostGeometryAngleType = boost::geometry::radian>
using GeoPoint2d = boost::geometry::model::d2::point_xy<double, boost::geometry::cs::geographic<BoostGeometryAngleType>>;

template <class BoostGeometryAngleType = boost::geometry::radian>
using GeoPolygon2d = boost::geometry::model::polygon<GeoPoint2d<BoostGeometryAngleType>>;

template <class BoostGeometryAngleType = boost::geometry::radian>
using GeoRing2d = typename boost::geometry::ring_type<GeoPolygon2d<BoostGeometryAngleType>>::type;
}
}
}
}
