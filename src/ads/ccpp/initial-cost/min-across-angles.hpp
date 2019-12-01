#pragma once

#include <tuple>
#include <utility>

#include <boost/concept/assert.hpp>
#include <boost/concept_check.hpp>

#include "ads/ccpp/optimal-direction/min-across-angles.hpp"
#include "ads/ccpp/turn-cost/turn-cost-concept.h"

namespace ads {
namespace ccpp {
namespace initial_cost {

template<class TurnCostCalculator>
class MinAcrossAngles
{
    quantity::Radians m_increment;
    TurnCostCalculator m_turnCalculator;

public:
    BOOST_CONCEPT_ASSERT((turn_cost::TurnCostConcept<TurnCostCalculator>));
    BOOST_CONCEPT_ASSERT((boost::Assignable<TurnCostCalculator>));

    MinAcrossAngles(TurnCostCalculator turnCalculator, const quantity::Radians increment)
        : m_increment(increment)
        , m_turnCalculator(turnCalculator)
    {
    }

    MinAcrossAngles(TurnCostCalculator turnCalculator, const quantity::Degrees increment = 1.0*units::Degree)
        : MinAcrossAngles(turnCalculator, static_cast<quantity::Radians>(increment))
    {
    }

    std::pair<double, quantity::Radians> operator()(const geometry::Polygon2d& poly) const
    {
        const static auto quarterTurn = static_cast<quantity::Radians>(units::Degree * 90);

        const auto optimalDirResult =
            optimal_direction::MinAcrossAngles<TurnCostCalculator>(m_turnCalculator, m_increment)(poly);

        // Return same cost, but normal to the optimal direction
        // as the sweep dir
        return {optimalDirResult.first, optimalDirResult.second + quarterTurn};
    }
};

}
}
}
