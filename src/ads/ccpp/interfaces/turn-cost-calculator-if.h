#pragma once

#include "ads/ccpp/typedefs.h"

namespace ads
{
namespace ccpp
{
namespace interfaces
{

class TurnCostCalculatorIf
{
  public:
    virtual ~TurnCostCalculatorIf() = default;

    // Wrappers for all segment types because you can't do virtual inheritance on template functions
    // and this library isn't using duck-typing
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
