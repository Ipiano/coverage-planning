#pragma once

#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

#include <boost/units/systems/si/plane_angle.hpp>
#include <boost/units/systems/si/io.hpp>

#include "ads/ccpp/typedefs.h"

namespace ads {
namespace ccpp {
namespace interfaces {

class OptimalDirectionCalculatorIf
{
public:
    virtual ~OptimalDirectionCalculatorIf() = default;

    /*!
     * \brief Approximates the optimal direction of travel for a field or portion of a field
     *
     * Since this is often a Hard Problem, an estimated best direction
     * will suffice.
     *
     * \param[in] field - Polygon representing the area to find a direction for
     *
     * \return Appximate best angle of travel for A+ swaths to minimize cost overall
     */
    virtual quantity::Radians calculateOptimalDirection(const geometry::Polygon2d& field) const = 0;
};

}
}
}
