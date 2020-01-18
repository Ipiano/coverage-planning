#include "u-shaped.h"

#include "ads/ccpp/typedefs.h"

namespace ads
{
namespace ccpp
{
namespace turn_cost
{

constexpr static double pi = static_cast<quantity::Radians>(units::Degree * 180).value();

UShaped::UShaped(const double turnWeight, const double headlandWeight1, const double headlandWeight2)
    : m_weight1(turnWeight), m_weight2(headlandWeight1), m_weight3(headlandWeight2)
{
}

double UShaped::_calculateTurnCost(const geometry::ConstReferringSegment2d& segment, const quantity::Radians travelAngle) const
{
    namespace bg = boost::geometry;

    const double dy           = bg::get<0, 1>(segment) - bg::get<1, 1>(segment);
    const double dx           = bg::get<0, 0>(segment) - bg::get<1, 0>(segment);
    const double segmentAngle = std::atan2(dy, dx);

    const double angleDelta    = fixAngle(travelAngle.value() - segmentAngle);
    const double segmentLength = bg::length(segment);

    // Headland term goes to 0 if direction of travel is parallel
    // to line segment. Original equation would have been w/tan(0) in that case,
    // which is a div by 0 error. But logically, there's no cost for edges you don't
    // turn around on.
    const double headlandTerm = angleDelta < 0.00001 ? 0 : segmentLength * std::abs(std::cos(angleDelta)) / 2.;
    const double turnTerm     = pi * segmentLength * std::sin(angleDelta) / 4.;

    return turnTerm * m_weight1 + headlandTerm * m_weight2 + headlandTerm * m_weight3;
}

double UShaped::fixAngle(double radians) const
{
    if (radians < -pi / 2)
    {
        radians = radians + pi * (-int((radians + pi / 2) / pi) + 1);
    }
    else if (radians >= pi / 2)
    {
        radians = radians - pi * int((radians + pi / 2) / pi);
    }
    return std::abs(radians);
}
}
}
}
