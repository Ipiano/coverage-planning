#pragma once

#include <tuple>
#include <utility>

#include <boost/concept/assert.hpp>
#include <boost/concept_check.hpp>

#include "ads/ccpp/optimal-direction/min-across-angles.h"
#include "ads/ccpp/interfaces/optimal-direction-calculator-if.h"

namespace ads
{
namespace ccpp
{
namespace initial_cost
{

class MinAcrossAngles
{
    const interfaces::OptimalDirectionCalculatorIf& m_directionCalculator;

  public:
    MinAcrossAngles(const interfaces::OptimalDirectionCalculatorIf& dirCalculator) : m_directionCalculator(dirCalculator) {}

    quantity::Radians calculateInitialDirection(const geometry::Polygon2d& poly) const
    {
        const static auto quarterTurn = static_cast<quantity::Radians>(units::Degree * 90);

        const auto optimalDirResult = m_directionCalculator.calculateOptimalDirection(poly);

        // Return same cost, but normal to the optimal direction
        // as the sweep dir
        return optimalDirResult + quarterTurn;
    }
};
}
}
}
