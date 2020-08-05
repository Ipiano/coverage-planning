#pragma once

#include "ads/ccpp/interfaces/optimal-direction-calculator-if.h"
#include "ads/ccpp/interfaces/turn-cost-calculator-if.h"
#include "ads/ccpp/typedefs.h"

namespace ads
{
namespace ccpp
{
namespace optimal_direction
{

/*!
 * \brief Optimal direction calculator which checks for the minimum cost across a range of angles
 */
class MinAcrossAngles : public interfaces::OptimalDirectionCalculatorIf
{
  public:
    /*!
     * \brief Initializes the MinAcrossAngles calculator with a specific turn cost calculator and angle increment
     * \param turnCalculator Calculator to use for determining cost of an edge at an angle
     * \param increment Amount in radians to increment the angle by when sweeping across the range
     */
    MinAcrossAngles(const interfaces::TurnCostCalculatorIf& turnCalculator, const quantity::Radians increment);

    /*!
     * \brief Initializes the MinAcrossAngles calculator with a specific turn cost calculator and angle increment
     * \param turnCalculator Calculator to use for determining cost of an edge at an angle
     * \param increment Amount in degrees to increment the angle by when sweeping across the range
     */
    MinAcrossAngles(const interfaces::TurnCostCalculatorIf& turnCalculator, const quantity::Degrees increment = 1 * units::Degree);

    /*!
     * \brief Calculates the optimal direction and cost of a polygon
     *
     * Checks across the range (0, 180) degrees, incrementing by the increment value given
     * during construction. Whichever angle checked has the smallest cost associated with it is returned.
     *
     * \param poly Shape to calculate cost of
     * \return Pair containing the optimal direction and cost value associated with it as returned by the TurnCostCalculatorIf
     */
    std::pair<quantity::Radians, double> calculateOptimalDirectionAndCost(const geometry::Polygon2d& poly) const override;

    /*!
     * \brief Calculates the optimal direction and cost of a dcel::Region
     *
     * Checks across the range (0, 180) degrees, incrementing by the increment value given
     * during construction. Whichever angle checked has the smallest cost associated with it is returned.
     *
     * \param dcelRegion Region to calculate cost of
     * \return Pair containing the optimal direction and cost value associated with it as returned by the TurnCostCalculatorIf
     */
    std::pair<quantity::Radians, double> calculateOptimalDirectionAndCost(const dcel::Region dcelRegion) const override;

    /*!
     * \brief Gets the cost of a specific edge
     *
     * This passes directly through to the underlying TurnCostCalculatorIf.
     *
     * \param p1 First point of the edge
     * \param p2 Second point of the edge
     * \param direction Angle in radians to calculate cost for
     * \return Cost value associated with the edge as returned by the TurnCostCalculatorIf
     */
    double edgeCost(const geometry::Point2d& p1, const geometry::Point2d& p2, quantity::Radians direction) const override;

    /*!
     * \brief Gets the cost of a dcel Region if traversed at a specific angle
     *
     * This is equivalent to summing the edgeCost for every edge in the region.
     *
     * \param dcelRegion Region to get the cost of
     * \param direction Angle in radians to calculate cost for
     * \return Cost value associated with the region as returned by the TurnCostCalculatorIf
     */
    double totalCost(const dcel::Region dcelRegion, quantity::Radians direction) const override;

  private:
    quantity::Radians m_increment;
    const interfaces::TurnCostCalculatorIf& m_turnCalculator;
};
}
}
}
