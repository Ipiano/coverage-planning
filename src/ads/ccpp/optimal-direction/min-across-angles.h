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

class MinAcrossAngles : public interfaces::OptimalDirectionCalculatorIf
{
  public:
    MinAcrossAngles(const interfaces::TurnCostCalculatorIf& turnCalculator, const quantity::Radians increment);
    MinAcrossAngles(const interfaces::TurnCostCalculatorIf& turnCalculator, const quantity::Degrees increment = 1 * units::Degree);

    quantity::Radians calculateOptimalDirection(const geometry::Polygon2d& poly) const override;

  private:
    quantity::Radians m_increment;
    const interfaces::TurnCostCalculatorIf& m_turnCalculator;
};
}
}
}
