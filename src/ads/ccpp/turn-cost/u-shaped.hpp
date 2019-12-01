#pragma once

#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/segment.hpp>
#include <boost/geometry/core/access.hpp>
#include <boost/geometry/algorithms/length.hpp>

#include <boost/units/systems/si/plane_angle.hpp>
#include <boost/units/systems/angle/degrees.hpp>
#include <boost/units/systems/si/io.hpp>

namespace ads {
namespace ccpp {
namespace turn_cost {

template<class Segment2d>
class UShaped
{
    typedef boost::geometry::point_type<Segment2d> Point2d;

    typedef boost::units::quantity<boost::units::si::plane_angle> AngleRad;
    typedef boost::units::quantity<boost::units::degree::plane_angle> AngleDeg;

    const double pi = static_cast<AngleRad>(boost::units::degree::degree * 180).value();

    double m_weight1;
    double m_weight2;
    double m_weight3;

    // Adjusts an angle to 0-90 degrees,
    // by first adding a multiple of pi to get to the range
    // [-90, 90] and then returning the absolute value
    double fixAngle(double radians) const
    {
        if(radians < -pi/2)
        {
            radians = radians + pi * (-int((radians+pi/2)/pi) + 1);
        }
        else if(radians >= pi/2)
        {
            radians = radians - pi * int((radians+pi/2)/pi);
        }
        return std::abs(radians);
    }

public:
    UShaped(const double turnWeight=1., const double headlandWeight1 = 1., const double headlandWeight2 = 1.)
        : m_weight1(turnWeight), m_weight2(headlandWeight1), m_weight3(headlandWeight2)
    {

    }

    double operator()(const Segment2d& segment, const AngleRad& travelAngle) const
    {
        namespace bg = boost::geometry;
        namespace bu = boost::units;

        const double dy = bg::get<0, 1>(segment) - bg::get<1, 1>(segment);
        const double dx = bg::get<0, 0>(segment) - bg::get<1, 0>(segment);
        const double segmentAngle = std::atan2(dy, dx);

        const double angleDelta = fixAngle(travelAngle.value() - segmentAngle);
        const double segmentLength = bg::length(segment);

        // Headland term goes to 0 if direction of travel is parallel
        // to line segment. Original equation would have been w/tan(0) in that case,
        // which is a div by 0 error. But logically, there's no cost for edges you don't
        // turn around on.
        const double headlandTerm = angleDelta < 0.00001 ? 0 : segmentLength * std::abs(std::cos(angleDelta)) / 2.;
        const double turnTerm = pi * segmentLength * std::sin(angleDelta) / 4.;

        return turnTerm * m_weight1 + headlandTerm * m_weight2 + headlandTerm * m_weight3;
    }
};

}
}
}
