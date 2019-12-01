#pragma once

#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

#include <boost/units/systems/si/plane_angle.hpp>
#include <boost/units/systems/si/io.hpp>

#include "ads/ccpp/typedefs.h"

namespace ads {
namespace ccpp {
namespace optimal_direction {

template<class OptimalDirectionImpl>
class OptimalDirectionConcept
{
    const OptimalDirectionImpl m_initialCost;

public:
    void constraints()
    {
        // Should have an () operator, taking a polygon,
        // returning the initial cost and the angle of travel for it
        std::pair<double, quantity::Radians> result = m_initialCost(geometry::Polygon2d());
        boost::ignore_unused(result);
    }
};


}
}
}
