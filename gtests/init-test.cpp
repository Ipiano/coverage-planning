#include "ads/ccpp/quickopp.h"
#include "ads/ccpp/polygon-decomposer/modified-trapezoidal/modified-trapezoidal.h"

#include <gtest/gtest.h>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

namespace bg = boost::geometry;
using namespace testing;
using namespace ads;

typedef bg::model::d2::point_xy<double> PointType;
typedef bg::model::polygon<PointType> PolygonType;

TEST(CCPPTests, CanInstantiate)
{
}
