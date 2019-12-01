#include "ads/ccpp/turn-cost/u-shaped.hpp"

#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/segment.hpp>
#include <boost/geometry.hpp>

#include <boost/units/systems/si/plane_angle.hpp>
#include <boost/units/systems/angle/degrees.hpp>
#include <boost/units/systems/si/io.hpp>

#include <gtest/gtest.h>

using namespace testing;
using namespace ads::ccpp;
using namespace boost::geometry;
using namespace boost::units;

typedef model::d2::point_xy<double> Point2d;
typedef model::segment<Point2d> Segment2d;

typedef quantity<si::plane_angle> radians;
radians toRad(const double degrees)
{
    return static_cast<radians>(boost::units::degree::degree * degrees);
}

TEST(CCPPTests, UTurnIs0WhenParallel)
{
    turn_cost::UShaped<Segment2d> calculator;

    double cost = calculator({{0, 0}, {0, 1}}, toRad(90));
    EXPECT_NEAR(cost, 0, 0.001);

    cost = calculator({{0, 0}, {1, 0}}, toRad(0));
    EXPECT_NEAR(cost, 0, 0.001);

    cost = calculator({{0, 0}, {1, 1}}, toRad(45));
    EXPECT_NEAR(cost, 0, 0.001);

    cost = calculator({{0, 0}, {-1, 1}}, toRad(135));
    EXPECT_NEAR(cost, 0, 0.001);
}

TEST(CCPPTests, UTurnIsUndirectional)
{
    turn_cost::UShaped<Segment2d> calculator;

    double cost = calculator({{0, 0}, {0, 1}}, toRad(90));
    EXPECT_NEAR(cost, 0, 0.001);

    cost = calculator({{0, 0}, {0, 1}}, toRad(-90));
    EXPECT_NEAR(cost, 0, 0.001);

    cost = calculator({{0, 0}, {0, 1}}, toRad(270));
    EXPECT_NEAR(cost, 0, 0.001);

    cost = calculator({{0, 0}, {0, 1}}, toRad(-270));
    EXPECT_NEAR(cost, 0, 0.001);

    cost = calculator({{0, 0}, {0, 1}}, toRad(90));
    EXPECT_NEAR(cost, 0, 0.001);

    cost = calculator({{0, 0}, {0, 1}}, toRad(-90));
    EXPECT_NEAR(cost, 0, 0.001);

    cost = calculator({{0, 0}, {0, 1}}, toRad(270));
    EXPECT_NEAR(cost, 0, 0.001);

    cost = calculator({{0, 0}, {0, 1}}, toRad(-270));
    EXPECT_NEAR(cost, 0, 0.001);

}
