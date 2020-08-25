#pragma once

#include <tuple>
#include <utility>

#include <boost/concept/assert.hpp>
#include <boost/concept_check.hpp>

#include "ads/ccpp/optimal-direction/min-across-angles.h"
#include "ads/ccpp/interfaces/optimal-direction-calculator-if.h"

namespace ads
{
namespace ccpp
{
namespace initial_cost
{

/*!
 * \brief Estimates the initial cost of a polygon by finding the min cost across a range of angles
 *
 * The QuickOpp algorithm starts by picking a direction to run the sweep-line while decomposing
 * the polygon. Due to the way the decomposition algorithm works, regions are created along
 * the perpendicular direction to the initial one; therefore it is important to chose an initial
 * direction that is likely to result in regions that are approximately the best direction for the
 * shape overall.
 *
 * MinAcrossAngles picks a direction by using an OptimalDirectionCalculatorIf to calculate
 * an optimal direction for the shape overall, and then returns the normal direction to that
 * overall best direction. This guarantees that most of the regions generated during decomposition
 * will be in the direction of that best overall, which is likely to produce the most long-thin regions
 * with few turns in them.
 */
class MinAcrossAngles
{
    const interfaces::OptimalDirectionCalculatorIf& m_directionCalculator;

  public:
    /*!
     * \brief Inits the MinAcrossAngles chooser with an optimal direction calculator
     * \param dirCalculator OptimalDirectionCalculatorIf instance to use for cost calculations
     */
    MinAcrossAngles(const interfaces::OptimalDirectionCalculatorIf& dirCalculator) : m_directionCalculator(dirCalculator) {}

    /*!
     * \brief Picks the initial direction for the sweep line
     *
     * Uses the OptimalDirectionCalculator to find the overall optimal direction for
     * the shape given.
     *
     * \param poly Shape to find initial direction for
     * \return Direction, in radians, of the normal to the lowest-cost direction, and cost of covering the field
     *          entirely that direction
     */
    std::pair<quantity::Radians, double> calculateInitialDirectionAndCost(const geometry::Polygon2d& poly) const
    {
        const static auto quarterTurn = static_cast<quantity::Radians>(units::Degree * 90);

        const auto initialDirectionAndCost = m_directionCalculator.calculateOptimalDirectionAndCost(poly);

        // Return same cost, but normal to the optimal direction
        // as the sweep dir
        return {initialDirectionAndCost.first + quarterTurn, initialDirectionAndCost.second};
    }
};
}
}
}
