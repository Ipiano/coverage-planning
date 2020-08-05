#pragma once

#include "ads/ccpp/typedefs.h"

namespace ads
{
namespace ccpp
{
namespace interfaces
{

/*!
 * \brief Interface for the cost function of the ccpp algorithm
 */
class TurnCostCalculatorIf
{
  public:
    virtual ~TurnCostCalculatorIf() = default;

    /*!
     * \brief Calculates the cost associated with turnarounds for an edge of a field
     *
     * This function is just a templated wrapper around _calculateTurnCost to provide support
     * in TurnCostCalculatorIf for arbitrary segment types. Classes inheriting this interface should
     * include `using TurnCostCalculatorIf::calculateTurnCost` somewhere in the class declaration's public
     * area in order to expose this function.
     *
     * \tparam SegmentT Some type indicating a line segment with members ::first and ::second containing points
     *
     * \param edge Edge to calculate cost for
     * \param swathDir Angle in radians to find cost of
     *
     * \returns double cost of the turn
     */
    template <class SegmentT> double calculateTurnCost(const SegmentT& edge, const quantity::Radians swathDir) const
    {
        return _calculateTurnCost(geometry::ConstReferringSegment2d(edge.first, edge.second), swathDir);
    }

  protected:
    /*!
     * \brief Calculates the cost associated with turnarounds for an edge of a field
     *
     * Must return the same value for an edge with the same points in the opposite order
     * or any direction that is the given one + a multiple of 180 degrees
     *
     * Cost is not required to have any specific units associated with it; however, it is expected
     * to be a monotonic function which increases as the cost associated with planting along
     * an edge increases.
     *
     * \param[in] edge An edge of a field boundary
     * \param[in] swathDir Direction of swathing (As if an A+ pattern)
     *
     * \return Total cost associated with all turnarounds on the edge for this direction
     */
    virtual double _calculateTurnCost(const geometry::ConstReferringSegment2d& edge, const quantity::Radians swathDir) const = 0;
};
}
}
}
