#pragma once

#include "ads/ccpp/typedefs.h"
#include "ads/ccpp/interfaces/turn-cost-calculator-if.h"

namespace ads
{
namespace ccpp
{

/*!
 * \brief Functor object used to sum the edge cost of all edges in a shape for a specific angle of travel
 *
 * Suitable for use in boost::for_each_segment or dcel::forEachSegment.
 */
struct AngleCostSum
{
    quantity::Radians m_angle;
    double m_totalCost;
    const interfaces::TurnCostCalculatorIf* m_calculator;

  public:
    /*!
     * \brief Initializes the summation functor with a target calculation function and angle of travel
     * \param calculator Cost calculator to use to get costs to sum
     * \param angle Target angle of travel to sum costs for
     */
    AngleCostSum(const interfaces::TurnCostCalculatorIf& calculator, const quantity::Radians angle)
        : m_angle(angle), m_totalCost(0), m_calculator(&calculator)
    {
    }

    template <class SegmentT> void operator()(const SegmentT& segment) { m_totalCost += m_calculator->calculateTurnCost(segment, m_angle); }

    /*!
     * \brief Gets the total cost
     * \return Total cost summed over all segments passed to the functor
     */
    double cost() const { return m_totalCost; }
};
}
}
