#include "u-shaped.h"

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

const double UShaped::pi = static_cast<AngleRad>(boost::units::degree::degree * 180).value();

double UShaped::fixAngle(double radians) const
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

UShaped::UShaped(const double turnWeight, const double headlandWeight1, const double headlandWeight2)
    : m_weight1(turnWeight), m_weight2(headlandWeight1), m_weight3(headlandWeight2)
{

}

}
}
}
