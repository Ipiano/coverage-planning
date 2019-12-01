#include "ads/ccpp/turn-cost/u-shaped.h"

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
    turn_cost::UShaped calculator;

    double cost = calculator(Segment2d{{0, 0}, {0, 1}}, toRad(90));
    EXPECT_NEAR(cost, 0, 0.001);

    cost = calculator(Segment2d{{0, 0}, {1, 0}}, toRad(0));
    EXPECT_NEAR(cost, 0, 0.001);

    cost = calculator(Segment2d{{0, 0}, {1, 1}}, toRad(45));
    EXPECT_NEAR(cost, 0, 0.001);

    cost = calculator(Segment2d{{0, 0}, {-1, 1}}, toRad(135));
    EXPECT_NEAR(cost, 0, 0.001);
}

TEST(CCPPTests, UTurnIsUndirectional)
{
    turn_cost::UShaped calculator;

    double cost = calculator(Segment2d{{0, 0}, {0, 1}}, toRad(90));
    EXPECT_NEAR(cost, 0, 0.001);

    cost = calculator(Segment2d{{0, 0}, {0, 1}}, toRad(-90));
    EXPECT_NEAR(cost, 0, 0.001);

    cost = calculator(Segment2d{{0, 0}, {0, 1}}, toRad(270));
    EXPECT_NEAR(cost, 0, 0.001);

    cost = calculator(Segment2d{{0, 0}, {0, 1}}, toRad(-270));
    EXPECT_NEAR(cost, 0, 0.001);

    cost = calculator(Segment2d{{0, 0}, {0, 1}}, toRad(90));
    EXPECT_NEAR(cost, 0, 0.001);

    cost = calculator(Segment2d{{0, 0}, {0, 1}}, toRad(-90));
    EXPECT_NEAR(cost, 0, 0.001);

    cost = calculator(Segment2d{{0, 0}, {0, 1}}, toRad(270));
    EXPECT_NEAR(cost, 0, 0.001);

    cost = calculator(Segment2d{{0, 0}, {0, 1}}, toRad(-270));
    EXPECT_NEAR(cost, 0, 0.001);
}

TEST(CCPPTests, UTurnPerpendicularIsOnlyTurn)
{
    turn_cost::UShaped calculator;

    Segment2d segment{{0, 0}, {0, 1}};
    double cost = calculator(segment, toRad(0));
    double expected = toRad(180).value() * double(boost::geometry::length(segment)) * std::sin(toRad(90).value()) / 4.;

    EXPECT_NEAR(cost, expected, 0.001);

    segment = {{0, 0}, {1, 1}};
    cost = calculator(segment, toRad(-45));
    expected = toRad(180).value() * double(boost::geometry::length(segment)) * std::sin(toRad(90).value()) / 4.;

    EXPECT_NEAR(cost, expected, 0.001);
}

TEST(CCPPTests, UTurnAtAngleMoreExpensiveThanPerpendicular)
{
    turn_cost::UShaped calculator;

    Segment2d segment{{0, 0}, {0, 1}};
    EXPECT_GT(calculator(segment, toRad(-45)), calculator(segment, toRad(0)));

    segment = {{0, 0}, {1, 1}};
    EXPECT_GT(calculator(segment, toRad(0)), calculator(segment, toRad(-45)));

}
