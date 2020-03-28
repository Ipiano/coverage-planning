#pragma once

#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

#include <boost/units/systems/si/plane_angle.hpp>
#include <boost/units/systems/si/io.hpp>

#include "ads/ccpp/typedefs.h"
#include "ads/ccpp/dcel.h"

namespace ads
{
namespace ccpp
{
namespace interfaces
{

/*!
 * \brief Approximates the optimal direction of travel for a field or portion of a field
 *
 * Since this is often a Hard Problem, an estimated best direction
 * will suffice.
 *
 * All calculations done by this class should use the same cost function.
 *
 * All angles returned should be in the range [0, PI)
 */
class OptimalDirectionCalculatorIf
{
  public:
    virtual ~OptimalDirectionCalculatorIf() = default;

    /*!
     * \brief Finds best direction for a specific shape with holes
     * \param[in] field Polygon to evaluate
     * \return Appximate best angle of travel for the given shape and cost of covering it that way
     */
    virtual std::pair<quantity::Radians, double> calculateOptimalDirectionAndCost(const geometry::Polygon2d& field) const = 0;

    /*!
     * \brief Finds best direction for a Doubly-Connected Edge List region
     * \param[in] dcelRegion DCEL region to evaluate
     * \return Appximate best angle of travel for the given region and cost of covering it that way
     */
    virtual std::pair<quantity::Radians, double> calculateOptimalDirectionAndCost(const dcel::region_t& dcelRegion) const = 0;

    /*!
     * \brief Gives a cost value for a region, if the direction of travel is a specfic angle
     *
     * The points should not be assumed to be given in any specific order
     *
     * \param[in] dcelRegion DCEL region to evaluate
     * \param[in] direction Direction of travel
     *
     * \return Some cost value
     */
    virtual double totalCost(const dcel::region_t& dcelRegion, quantity::Radians direction) const = 0;

    /*!
     * \brief Gives a cost value for a specific edge, if the direction of travel is a specfic angle
     *
     * The points should not be assumed to be given in any specific order
     *
     * \param[in] p1 First end of the edge
     * \param[in] p2 Second end of the edge
     * \param[in] direction Direction of travel
     *
     * \return Some cost value
     */
    virtual double edgeCost(const geometry::Point2d& p1, const geometry::Point2d& p2, quantity::Radians direction) const = 0;
};
}
}
}
