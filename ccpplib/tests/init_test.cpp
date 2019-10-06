#include "ccpplib/boustrophedon_decomposer.h"
#include "ccpplib/ccpp.h"

#include <gtest/gtest.h>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

namespace bg = boost::geometry;
using namespace testing;

typedef bg::model::d2::point_xy<double> PointType;
typedef bg::model::polygon<PointType> PolygonType;

TEST(CCPPTests, CanInstantiate)
{
  ccpp::BoustrophedonDecomposer<PolygonType> decomposer;
  ccpp::SwathPlanner<PolygonType> planner(&decomposer);
}
