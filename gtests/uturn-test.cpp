#include "ads/ccpp/turn-cost/u-shaped.h"

#include <gtest/gtest.h>

using namespace testing;
using namespace ads::ccpp;
using namespace boost::geometry;

typedef model::d2::point_xy<double> Point2d;
typedef model::segment<Point2d> Segment2d;

quantity::Radians toRad(const double degrees)
{
    return static_cast<quantity::Radians>(units::Degree * degrees);
}

//! \test Test that U-Turn cost function returns 0 when given edge is parallel to direction of travel
TEST(CCPPTests, UTurnIs0WhenParallel)
{
    turn_cost::UShaped calculator(1, 1, 1);

    double cost = calculator.calculateTurnCost(Segment2d{{0, 0}, {0, 1}}, toRad(90));
    EXPECT_NEAR(cost, 0, 0.001);

    cost = calculator.calculateTurnCost(Segment2d{{0, 0}, {1, 0}}, toRad(0));
    EXPECT_NEAR(cost, 0, 0.001);

    cost = calculator.calculateTurnCost(Segment2d{{0, 0}, {1, 1}}, toRad(45));
    EXPECT_NEAR(cost, 0, 0.001);

    cost = calculator.calculateTurnCost(Segment2d{{0, 0}, {-1, 1}}, toRad(135));
    EXPECT_NEAR(cost, 0, 0.001);
}

//! \test Test that U-Turn cost function returns the same value when either the
//! direction of travel is flipped 180 degrees
TEST(CCPPTests, UTurnIsUndirectional)
{
    turn_cost::UShaped calculator(1, 1, 1);

    double cost = calculator.calculateTurnCost(Segment2d{{0, 0}, {0, 1}}, toRad(90));
    EXPECT_NEAR(cost, 0, 0.001);

    cost = calculator.calculateTurnCost(Segment2d{{0, 0}, {0, 1}}, toRad(-90));
    EXPECT_NEAR(cost, 0, 0.001);

    cost = calculator.calculateTurnCost(Segment2d{{0, 0}, {0, 1}}, toRad(270));
    EXPECT_NEAR(cost, 0, 0.001);

    cost = calculator.calculateTurnCost(Segment2d{{0, 0}, {0, 1}}, toRad(-270));
    EXPECT_NEAR(cost, 0, 0.001);

    cost = calculator.calculateTurnCost(Segment2d{{0, 0}, {0, 1}}, toRad(90));
    EXPECT_NEAR(cost, 0, 0.001);

    cost = calculator.calculateTurnCost(Segment2d{{0, 0}, {0, 1}}, toRad(-90));
    EXPECT_NEAR(cost, 0, 0.001);

    cost = calculator.calculateTurnCost(Segment2d{{0, 0}, {0, 1}}, toRad(270));
    EXPECT_NEAR(cost, 0, 0.001);

    cost = calculator.calculateTurnCost(Segment2d{{0, 0}, {0, 1}}, toRad(-270));
    EXPECT_NEAR(cost, 0, 0.001);
}

//! \test Test that the U-Turn cost function returns the circumference of a half-circle when
//! direction of travel is perpendicular to the edge and all weights are 1
TEST(CCPPTests, UTurnPerpendicularIsOnlyTurn)
{
    turn_cost::UShaped calculator(1, 1, 1);

    Segment2d segment{{0, 0}, {0, 1}};
    double cost     = calculator.calculateTurnCost(segment, toRad(0));
    double expected = toRad(180).value() * double(boost::geometry::length(segment)) * std::sin(toRad(90).value()) / 4.;

    EXPECT_NEAR(cost, expected, 0.001);

    segment  = {{0, 0}, {1, 1}};
    cost     = calculator.calculateTurnCost(segment, toRad(-45));
    expected = toRad(180).value() * double(boost::geometry::length(segment)) * std::sin(toRad(90).value()) / 4.;

    EXPECT_NEAR(cost, expected, 0.001);
}

//! \test Test that the U-Turn cost function returns a higher value for non-perpendicular
//! directions of travel than perpendicular ones.
TEST(CCPPTests, UTurnAtAngleMoreExpensiveThanPerpendicular)
{
    turn_cost::UShaped calculator(1, 1, 1);

    Segment2d segment{{0, 0}, {0, 1}};
    EXPECT_GT(calculator.calculateTurnCost(segment, toRad(-45)), calculator.calculateTurnCost(segment, toRad(0)));

    segment = {{0, 0}, {1, 1}};
    EXPECT_GT(calculator.calculateTurnCost(segment, toRad(0)), calculator.calculateTurnCost(segment, toRad(-45)));
}
