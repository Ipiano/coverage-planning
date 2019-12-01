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

#include "ads/ccpp/optimal-direction/min-across-angles.hpp"
#include "ads/ccpp/turn-cost/turn-cost-concept.hpp"

namespace ads {
namespace ccpp {
namespace initial_cost {

template<class TurnCostCalculator, class Polygon2d>
class MinAcrossAngles
{
    typedef boost::units::quantity<boost::units::si::plane_angle> AngleRad;
    typedef boost::units::quantity<boost::units::degree::plane_angle> AngleDeg;

    typedef boost::geometry::point_type<Polygon2d> Point2d;
    typedef boost::geometry::model::referring_segment<Point2d> Segment2d;

    AngleRad m_increment;
    TurnCostCalculator m_turnCalculator;

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
        namespace bu = boost::units;
        const static auto quarterTurn = static_cast<AngleRad>(bu::degree::degree * 90);

        const auto optimalDirResult =
            optimal_direction::MinAcrossAngles<TurnCostCalculator, Polygon2d>(m_turnCalculator, m_increment)(poly);

        // Return same cost, but normal to the optimal direction
        // as the sweep dir
        return {optimalDirResult.first, optimalDirResult.second + quarterTurn};
    }
};

}
}
}
