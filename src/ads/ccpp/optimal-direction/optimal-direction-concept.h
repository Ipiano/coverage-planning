#pragma once

#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

#include <boost/units/systems/si/plane_angle.hpp>
#include <boost/units/systems/si/io.hpp>

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
        typedef boost::geometry::model::d2::point_xy<double> Point2d;
        typedef boost::geometry::model::polygon<Point2d> Polygon2d;
        typedef boost::units::quantity<boost::units::si::plane_angle> AngleRad;

        // Should have an () operator, taking a polygon,
        // returning the initial cost and the angle of travel for it
        std::pair<double, AngleRad> result = m_initialCost(Polygon2d());
        boost::ignore_unused(result);
    }
};


}
}
}
