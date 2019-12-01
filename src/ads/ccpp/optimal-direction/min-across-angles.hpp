#pragma once

#include <tuple>
#include <utility>
#include <iostream>

#include <boost/concept/assert.hpp>
#include <boost/concept_check.hpp>

#include <boost/geometry/core/exterior_ring.hpp>

#include "ads/ccpp/turn-cost/turn-cost-concept.h"
#include "ads/ccpp/typedefs.h"

namespace ads {
namespace ccpp {
namespace optimal_direction {

template<class TurnCostCalculator>
class MinAcrossAngles
{
    quantity::Radians m_increment;
    TurnCostCalculator m_turnCalculator;

    struct AngleCostSum
    {
        quantity::Radians m_angle;
        double m_totalCost;
        TurnCostCalculator m_calculator;

    public:
        AngleCostSum(TurnCostCalculator calculator, const quantity::Radians angle)
            : m_angle(angle), m_totalCost(0), m_calculator(calculator){}

        template<class SegmentT>
        void operator()(const SegmentT& segment)
        {
            m_totalCost += m_calculator(segment, m_angle);
        }

        double cost() const
        {
            return m_totalCost;
        }
    };

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
        const static auto maxAngle = static_cast<quantity::Radians>(180*units::Degree);

        std::pair<double, quantity::Radians> bestResult(-1, units::Radian*0);

        const auto& outerRing = boost::geometry::exterior_ring(poly);
        if(outerRing.size() < 3)
            return bestResult;

        for(quantity::Radians currAngle = units::Radian*0; currAngle < maxAngle; currAngle += m_increment)
        {
            const auto costSum = boost::geometry::for_each_segment(outerRing, AngleCostSum(m_turnCalculator, currAngle));

            std::cout << currAngle << ": " << costSum.cost() << std::endl;

            if(costSum.cost() < bestResult.first || bestResult.first < 0)
            {
                bestResult = {costSum.cost(), currAngle};
            }
        }

        return bestResult;
    }
};

}
}
}
