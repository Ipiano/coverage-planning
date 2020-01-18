#include "min-across-angles.h"

#include "ads/ccpp/angle-cost-sum.hpp"

namespace ads {
namespace ccpp {
namespace optimal_direction {

MinAcrossAngles::MinAcrossAngles(const interfaces::TurnCostCalculatorIf& turnCalculator, const quantity::Radians increment)
        : m_increment(increment)
        , m_turnCalculator(turnCalculator)
    {
    }

    MinAcrossAngles::MinAcrossAngles(const interfaces::TurnCostCalculatorIf& turnCalculator, const quantity::Degrees increment)
        : MinAcrossAngles(turnCalculator, static_cast<quantity::Radians>(increment))
    {
    }

    quantity::Radians MinAcrossAngles::calculateOptimalDirection(const geometry::Polygon2d& poly) const
    {
        const static auto maxAngle = static_cast<quantity::Radians>(180*units::Degree);

        std::pair<double, quantity::Radians> bestResult(-1, units::Radian*0);

        const auto& outerRing = boost::geometry::exterior_ring(poly);
        if(outerRing.size() < 3)
            return bestResult.second;

        for(quantity::Radians currAngle = units::Radian*0; currAngle < maxAngle; currAngle += m_increment)
        {
            const auto costSum = boost::geometry::for_each_segment(poly, AngleCostSum(m_turnCalculator, currAngle));

            if(costSum.cost() < bestResult.first || bestResult.first < 0)
            {
                bestResult = {costSum.cost(), currAngle};
            }
        }

        return bestResult.second;
    }
}
}
}
