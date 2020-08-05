#pragma once

#include "ads/ccpp/typedefs.h"
#include "ads/ccpp/interfaces/turn-cost-calculator-if.h"

namespace ads
{
namespace ccpp
{
namespace turn_cost
{

/*!
 * \brief Calculates the cost of an edge using a standard U-Shaped turnaround
 *
 * This calculator assumes that when an edge is reached the driver will
 * * Continue forward until the entire implement is outside the edge
 * * Turn around, following a U shape that is exactly the width of the implement
 * * Continue forward along a line that is parallel to the incoming line.
 *
 * The cost function for an edge is taken directly from the original QuickOpp paper,
 * and is defined as
 *
 * \f[
 * A*(L*|cos(|\theta-\beta|)| / 2) + B*(L*|cos(|\theta-\beta|)| / 2) + C*(\pi L *sin(|\theta-\beta|)/4)
 * \f]
 *
 * Where \f$ \theta \f$ is the angle of the edge and \f$ \beta \f$ is the target angle for the swath being
 * driven (so \f$ \theta - \beta \f$ is the relative angle).
 *
 * You'll notice that the first two terms of this function are the same; these represent the cost of traveling
 * into the turn and back out after the turn; they are separated so that different weights (\f$ A \f$ and \f$ B \f$ respectively)
 * could be associated with each of them. The third term is the cost of the actual turn itself, and is weighted by \f$ C \f$.
 */
class UShaped : public interfaces::TurnCostCalculatorIf
{
  public:
    /*!
     * \brief Initializes a UShaped calculator with default weights
     *
     * The default weights put the cost of turning around significantly higher than the
     * cost of traveling into and out of the turn.
     */
    UShaped() : UShaped(2, 0.5, 0.5) {}

    /*!
     * \brief Initializes a UShaped calculator with specific weights
     *
     * \param turnWeight Weight applied to the actual turn portion of the cost
     * \param headlandWeight1 Weight applied to the straight path driven into the turn
     * \param headlandWeight2 Weight applied to the straight path driven out of the turn
     */
    UShaped(const double turnWeight, const double headlandWeight1, const double headlandWeight2);

  protected:
    double _calculateTurnCost(const geometry::ConstReferringSegment2d& segment, const quantity::Radians travelAngle) const override;

  private:
    double m_weight1;
    double m_weight2;
    double m_weight3;

    //! Adjusts an angle to 0-90 degrees,
    //! by first adding a multiple of pi to get to the range
    //! [-90, 90] and then returning the absolute value
    double fixAngle(double radians) const;
};
}
}
}
