#include "min-across-angles.h"

#include "ads/ccpp/angle-cost-sum.hpp"

namespace ads
{
namespace ccpp
{
namespace optimal_direction
{

MinAcrossAngles::MinAcrossAngles(const interfaces::TurnCostCalculatorIf& turnCalculator, const quantity::Radians increment)
    : m_increment(increment), m_turnCalculator(turnCalculator)
{
}

MinAcrossAngles::MinAcrossAngles(const interfaces::TurnCostCalculatorIf& turnCalculator, const quantity::Degrees increment)
    : MinAcrossAngles(turnCalculator, static_cast<quantity::Radians>(increment))
{
}

template <class SumOp> SumOp for_each_segment(const geometry::Polygon2d& poly, SumOp sum)
{
    return boost::geometry::for_each_segment(poly, sum);
}

template <class SumOp> SumOp for_each_segment(const dcel::region_t& region, SumOp sum)
{
    return dcel::for_each_segment(region, sum);
}

template <class InputShape>
double cost(const InputShape& shape, const quantity::Radians angle, const interfaces::TurnCostCalculatorIf& turnCalculator)
{
    return optimal_direction::for_each_segment(shape, AngleCostSum(turnCalculator, angle)).cost();
}

template <class InputShape>
std::pair<quantity::Radians, double> calculateOptimalDirectionAndCost(const InputShape& shape, const quantity::Radians angleIncrement,
                                                                      const interfaces::TurnCostCalculatorIf& turnCalculator)
{
    const static auto maxAngle = static_cast<quantity::Radians>(180 * units::Degree);

    std::pair<quantity::Radians, double> bestResult(units::Radian * 0, -1);

    for (quantity::Radians currAngle = units::Radian * 0; currAngle < maxAngle; currAngle += angleIncrement)
    {
        const auto costSum = optimal_direction::cost(shape, currAngle, turnCalculator);

        if (costSum < bestResult.second || bestResult.second < 0)
        {
            bestResult = {currAngle, costSum};
        }
    }

    return bestResult;
}

std::pair<quantity::Radians, double> MinAcrossAngles::calculateOptimalDirectionAndCost(const geometry::Polygon2d& poly) const
{
    return optimal_direction::calculateOptimalDirectionAndCost(poly, m_increment, m_turnCalculator);
}

std::pair<quantity::Radians, double> MinAcrossAngles::calculateOptimalDirectionAndCost(const dcel::region_t& dcelRegion) const
{
    return optimal_direction::calculateOptimalDirectionAndCost(dcelRegion, m_increment, m_turnCalculator);
}

double MinAcrossAngles::totalCost(const dcel::region_t& dcelRegion, quantity::Radians direction) const
{
    return optimal_direction::cost(dcelRegion, direction, m_turnCalculator);
}

double MinAcrossAngles::edgeCost(const geometry::Point2d& p1, const geometry::Point2d& p2, quantity::Radians direction) const
{
    const geometry::ConstReferringSegment2d segment(p1, p2);
    return m_turnCalculator.calculateTurnCost(segment, direction);
}

}
}
}
