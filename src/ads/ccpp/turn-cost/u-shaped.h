#pragma once

#include "ads/ccpp/typedefs.h"
#include "ads/ccpp/interfaces/turn-cost-calculator-if.h"

namespace ads
{
namespace ccpp
{
namespace turn_cost
{

class UShaped : public interfaces::TurnCostCalculatorIf
{
  public:
    UShaped() : UShaped(2, 0.5, 0.5) {}
    UShaped(const double turnWeight, const double headlandWeight1, const double headlandWeight2);

  protected:
    double _calculateTurnCost(const geometry::ConstReferringSegment2d& segment, const quantity::Radians travelAngle) const override;

  private:
    double m_weight1;
    double m_weight2;
    double m_weight3;

    // Adjusts an angle to 0-90 degrees,
    // by first adding a multiple of pi to get to the range
    // [-90, 90] and then returning the absolute value
    double fixAngle(double radians) const;
};
}
}
}
