#pragma once

#include <tuple>
#include <utility>

#include <boost/concept/assert.hpp>
#include <boost/concept_check.hpp>

#include <boost/units/systems/si/plane_angle.hpp>
#include <boost/units/systems/angle/degrees.hpp>
#include <boost/units/systems/si/io.hpp>

#include <boost/geometry/core/exterior_ring.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

#include "ads/ccpp/turn-cost/turn-cost-concept.hpp"

namespace ads {
namespace ccpp {
namespace optimal_direction {

template<class TurnCostCalculator, class Polygon2d>
class MinAcrossAngles
{
    typedef boost::units::quantity<boost::units::si::plane_angle> AngleRad;
    typedef boost::units::quantity<boost::units::degree::plane_angle> AngleDeg;

    typedef boost::geometry::point_type<Polygon2d> Point2d;
    typedef boost::geometry::model::referring_segment<Point2d> Segment2d;

    AngleRad m_increment;
    TurnCostCalculator m_turnCalculator;

    struct AngleCostSum
    {
        AngleRad m_angle;
        double m_totalCost;
        TurnCostCalculator m_calculator;

    public:
        AngleCostSum(TurnCostCalculator calculator, const AngleRad angle)
            : m_angle(angle), m_totalCost(0), m_calculator(calculator){}

        void operator()(const Segment2d& segment)
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

    MinAcrossAngles(TurnCostCalculator turnCalculator, const AngleRad increment)
        : m_increment(increment)
        , m_turnCalculator(turnCalculator)
    {
    }

    MinAcrossAngles(TurnCostCalculator turnCalculator, const AngleDeg increment = 1.0*boost::units::degree::degrees)
        : MinAcrossAngles(turnCalculator, static_cast<AngleRad>(increment))
    {
    }

    std::pair<double, AngleRad> operator()(const Polygon2d& poly) const
    {
        using namespace boost::units::si;
        using namespace boost::units::degree;

        const static auto maxAngle = static_cast<AngleRad>(180*degree);

        std::pair<double, AngleRad> bestResult(-1, radian*0);

        const auto& outerRing = boost::geometry::exterior_ring(poly);
        if(outerRing.size() < 3)
            return bestResult;

        for(AngleRad currAngle = radian*0; currAngle < maxAngle; currAngle += m_increment)
        {
            const auto costSum = boost::geometry::for_each_segment(outerRing, AngleCostSum(m_turnCalculator, currAngle));

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
